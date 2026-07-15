/*
 *  VHDL code generation for processes.
 *
 *  Copyright (C) 2008-2021  Nick Gasson (nick@nickg.me.uk)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "vhdl_target.h"
#include "vhdl_element.hh"
#include "vhdl_syntax.hh"
#include "state.hh"

#include <iostream>
#include <cassert>
#include <sstream>
#include <algorithm>
#include <vector>
#include <set>
#include <utility>

// ---------------------------------------------------------------------------
// Shadow blocking-target signals with process variables.
//
// iverilog emits Verilog blocking assignments (`=`) as VHDL non-blocking
// signal assignments (`<=`) with an explicit `wait for 0 ns;` whenever a
// later statement in the same process reads the target.  This is the only
// way to recover the read-after-write semantics with signals.
//
// The trouble is that `wait for 0 ns;` introduces a delta cycle in the
// middle of an always-block.  If the process resets a signal at the top
// of the block (`signal <= default;`) and then a case branch overrides
// the signal AFTER the wait, the default value commits at the wait and
// the override commits one delta later — producing two events per
// iteration.  When the process is sensitive to its own outputs (as is
// the case for an always-comb translated this way), the second event
// re-fires the process and we never converge.  See livelock_test6.vhd.
//
// The fix is to shadow each blocking-target signal with a process-local
// variable.  Writes go to the variable (`v_sig := …`), reads come from
// the variable (`v_sig…`), and a single `sig <= v_sig` at the end of the
// process commits the final value.  No `wait for 0 ns;` is needed
// because variables hold their values across statements without delta
// cycles, so reads always see the most-recent in-process write.
// ---------------------------------------------------------------------------

namespace {

struct WriteInfo {
   bool found = false;
   bool ambiguous = false;
   vhdl_expr *slice = NULL;
   unsigned slice_width = 0;
};

// Does this body suspend on its own, via an explicit wait at the top level of
// the process? If not, the process is driven by a sensitivity list whose
// implicit wait sits at the END of the body -- see the seed placement below.
// Only a top-level wait counts: one buried in a branch might not be reached on
// every iteration, so a loop around the body could still spin.
static bool body_has_toplevel_wait(stmt_container *body)
{
   stmt_container::stmt_list_t &stmts = body->get_stmts();
   for (stmt_container::stmt_list_t::iterator it = stmts.begin();
        it != stmts.end(); ++it) {
      if (dynamic_cast<vhdl_wait_stmt*>(*it))
         return true;
   }
   return false;
}

static bool same_const_int(vhdl_expr *a, vhdl_expr *b)
{
   vhdl_const_int *ca = dynamic_cast<vhdl_const_int*>(a);
   vhdl_const_int *cb = dynamic_cast<vhdl_const_int*>(b);
   return ca && cb && ca->get_value() == cb->get_value();
}

static void find_write_info_recurse(vhdl_seq_stmt *s,
                                    const std::string &name,
                                    WriteInfo &info);

static void find_write_info(stmt_container *c,
                            const std::string &name,
                            WriteInfo &info)
{
   for (vhdl_seq_stmt *s : c->get_stmts()) {
      find_write_info_recurse(s, name, info);
      if (info.ambiguous)
         return;
   }
}

static void find_write_info_recurse(vhdl_seq_stmt *s,
                                    const std::string &name,
                                    WriteInfo &info)
{
   if (info.ambiguous)
      return;
   if (vhdl_nbassign_stmt *nb = dynamic_cast<vhdl_nbassign_stmt*>(s)) {
      if (nb->get_lhs()->get_name() == name) {
         vhdl_expr *slice = nb->get_lhs()->get_slice();
         unsigned w = nb->get_lhs()->get_slice_width();
         if (!info.found) {
            info.found = true;
            info.slice = slice;
            info.slice_width = w;
         } else {
            // Subsequent write: must target the same slice or we bail.
            bool same;
            if (info.slice == NULL && slice == NULL)
               same = true;
            else
               same = (info.slice_width == w
                       && same_const_int(info.slice, slice));
            if (!same)
               info.ambiguous = true;
         }
      }
   }
   std::vector<stmt_container*> subs;
   s->get_sub_containers(subs);
   for (stmt_container *sc : subs)
      find_write_info(sc, name, info);
}

static void mark_var_assigns(stmt_container *c, const std::string &var_name);

static void mark_var_assigns_recurse(vhdl_seq_stmt *s,
                                     const std::string &var_name)
{
   if (vhdl_nbassign_stmt *nb = dynamic_cast<vhdl_nbassign_stmt*>(s)) {
      if (nb->get_lhs()->get_name() == var_name)
         nb->set_emit_as_var();
   }
   std::vector<stmt_container*> subs;
   s->get_sub_containers(subs);
   for (stmt_container *sc : subs)
      mark_var_assigns(sc, var_name);
}

static void mark_var_assigns(stmt_container *c, const std::string &var_name)
{
   for (vhdl_seq_stmt *s : c->get_stmts())
      mark_var_assigns_recurse(s, var_name);
}

static void remove_wait_for_0(stmt_container *body)
{
   stmt_container::stmt_list_t &stmts = body->get_stmts();
   stmt_container::stmt_list_t::iterator it = stmts.begin();
   while (it != stmts.end()) {
      vhdl_wait_stmt *w = dynamic_cast<vhdl_wait_stmt*>(*it);
      if (w && w->get_type() == VHDL_WAIT_FOR0) {
         delete *it;
         it = stmts.erase(it);
      } else {
         std::vector<stmt_container*> subs;
         (*it)->get_sub_containers(subs);
         for (stmt_container *sub : subs)
            remove_wait_for_0(sub);
         ++it;
      }
   }
}

static vhdl_expr *clone_slice(vhdl_expr *e)
{
   if (e == NULL)
      return NULL;
   if (vhdl_const_int *ci = dynamic_cast<vhdl_const_int*>(e))
      return new vhdl_const_int(ci->get_value());
   return NULL;
}

static void shadow_blocking_targets(vhdl_process *vhdl_proc, vhdl_entity *ent)
{
   const std::set<std::string> &targets = vhdl_proc->get_blocking_targets();
   if (targets.empty())
      return;

   stmt_container *body = vhdl_proc->get_container();
   vhdl_scope *proc_scope = vhdl_proc->get_scope();
   vhdl_scope *arch_scope = ent->get_arch()->get_scope();

   // `v_sig := sig;` seeds, hoisted out of the body loop below.
   std::list<vhdl_seq_stmt*> seeds;

   for (std::set<std::string>::const_iterator tit = targets.begin();
        tit != targets.end(); ++tit) {
      const std::string &sig_name = *tit;

      vhdl_decl *sig_decl = arch_scope->get_decl(sig_name);
      if (sig_decl == NULL)
         continue;
      if (sig_decl->assignment_type() != vhdl_decl::ASSIGN_NONBLOCK)
         continue;  // already a variable — nothing to do

      WriteInfo info;
      find_write_info(body, sig_name, info);
      if (!info.found || info.ambiguous)
         continue;  // skip mixed-slice cases for now

      // If the write has a slice we don't know how to clone, skip.
      vhdl_expr *commit_lhs_slice = NULL;
      vhdl_expr *commit_rhs_slice = NULL;
      if (info.slice) {
         commit_lhs_slice = clone_slice(info.slice);
         commit_rhs_slice = clone_slice(info.slice);
         if (commit_lhs_slice == NULL || commit_rhs_slice == NULL) {
            delete commit_lhs_slice;
            delete commit_rhs_slice;
            continue;
         }
      }

      std::string var_name = "v_" + sig_name;
      while (proc_scope->have_declared(var_name))
         var_name += "_";

      const vhdl_type *src_type = sig_decl->get_type();
      vhdl_var_decl *var_decl =
         new vhdl_var_decl(var_name, new vhdl_type(*src_type));
      proc_scope->add_decl(var_decl);

      // Rename all in-body refs of sig_name to var_name.  This rewrites
      // both reads (RHS, conditions, etc.) and the LHS of nbassign stmts.
      vhdl_var_set_t reads, writes;
      body->find_vars(reads, writes);
      for (vhdl_var_set_t::iterator rit = reads.begin();
           rit != reads.end(); ++rit) {
         if ((*rit)->get_name() == sig_name)
            (*rit)->set_name(var_name);
      }
      for (vhdl_var_set_t::iterator wit = writes.begin();
           wit != writes.end(); ++wit) {
         if ((*wit)->get_name() == sig_name)
            (*wit)->set_name(var_name);
      }

      // The nbassign statements whose LHS now refers to the variable must
      // emit as `:=` rather than `<=`.
      mark_var_assigns(body, var_name);

      // Seed the shadow: `v_sig := sig;`. Collected here and emitted ONCE
      // before the body loop (see below) rather than prepended into the body.
      {
         vhdl_var_ref *init_lhs =
            new vhdl_var_ref(var_name, new vhdl_type(*src_type));
         vhdl_var_ref *init_rhs =
            new vhdl_var_ref(sig_name, new vhdl_type(*src_type));
         seeds.push_back(new vhdl_assign_stmt(init_lhs, init_rhs));
      }

      // Append `sig(slice) <= v_sig(slice);` just before the trailing
      // wait_on (or at end if no trailing wait).
      {
         vhdl_var_ref *lhs =
            new vhdl_var_ref(sig_name, new vhdl_type(*src_type));
         if (commit_lhs_slice)
            lhs->set_slice(commit_lhs_slice, info.slice_width);
         vhdl_var_ref *rhs =
            new vhdl_var_ref(var_name, new vhdl_type(*src_type));
         if (commit_rhs_slice)
            rhs->set_slice(commit_rhs_slice, info.slice_width);
         vhdl_nbassign_stmt *commit = new vhdl_nbassign_stmt(lhs, rhs);

         stmt_container::stmt_list_t &stmts = body->get_stmts();
         stmt_container::stmt_list_t::iterator wait_pos = stmts.end();
         for (stmt_container::stmt_list_t::iterator it = stmts.begin();
              it != stmts.end(); ++it) {
            vhdl_wait_stmt *w = dynamic_cast<vhdl_wait_stmt*>(*it);
            if (w && (w->get_type() == VHDL_WAIT_ON
                      || w->get_type() == VHDL_WAIT_INDEF))
               wait_pos = it;
         }
         if (wait_pos != stmts.end())
            stmts.insert(wait_pos, commit);
         else
            stmts.push_back(commit);
      }
   }

   // wait_for_0 stmts are no longer needed: blocking-read semantics are
   // now expressed through variables, which see all in-process writes
   // without a delta cycle.
   remove_wait_for_0(body);

   // Where the seed goes depends on how the process suspends.
   //
   // A process that suspends on its OWN explicit wait (`always #10 clk = ~clk`
   // -> `wait for 10 ms;`) re-runs its body from the top immediately after the
   // trailing `sig <= v_sig;` -- in the SAME delta, before that update has
   // settled. A seed at the top then re-reads the pre-write value, clobbers the
   // shadow, and the next write is a no-op: the clock toggled only every OTHER
   // period. For these, seeding is INITIALISATION, so hoist it out and loop the
   // body -- the standard idiom:
   //
   //   process is variable v_clk : logic3d; begin
   //     v_clk := clk;
   //     loop  wait for 10 ms;  v_clk := not v_clk;  clk <= v_clk;  end loop;
   //   end process;
   //
   // The shadow then just carries the last written value across iterations,
   // which is what a blocking-assignment target should do.
   //
   // A process with a SENSITIVITY LIST has no wait of its own: its implicit
   // wait is at the END of the body. It therefore does genuinely suspend before
   // re-running, so a seed at the top reads a settled value and is correct --
   // and wrapping its body in a loop would trap it so the implicit wait were
   // never reached, spinning forever. Keep the seed at the top for those.
   if (!seeds.empty()) {
      if (body_has_toplevel_wait(body)) {
         vhdl_loop_stmt *lp = new vhdl_loop_stmt;
         lp->get_container()->move_stmts_from(body);  // body (incl. commits) -> loop
         for (std::list<vhdl_seq_stmt*>::iterator it = seeds.begin();
              it != seeds.end(); ++it)
            body->add_stmt(*it);
         body->add_stmt(lp);
      }
      else {
         for (std::list<vhdl_seq_stmt*>::iterator it = seeds.begin();
              it != seeds.end(); ++it)
            body->prepend_stmt(*it);
      }
   }
}

}  // namespace

// Signals whose initial value was hoisted to a declaration default.
// These signals should have their first assignment skipped in the
// initial process to avoid creating a second driver.
static std::set<ivl_signal_t> g_hoisted_signals;

bool is_hoisted_signal(ivl_signal_t sig)
{
   return g_hoisted_signals.count(sig) > 0;
}

void clear_hoisted_signal(ivl_signal_t sig)
{
   g_hoisted_signals.erase(sig);
}

/*
 * Check if an initial process consists only of immediate assignments
 * with no delays, waits, loops, or conditionals -- i.e. it only
 * assigns values at time zero and then terminates.
 *
 * If so, collect the signal/value pairs so they can be applied as
 * signal declaration defaults instead of being emitted as a separate
 * process (which would create an extra driver and cause 'U' due to
 * resolution).
 */
struct init_assign_t {
   ivl_signal_t sig;
   ivl_expr_t value;
};

static bool is_time_zero_only(ivl_statement_t stmt,
                              std::vector<init_assign_t> &assigns)
{
   if (!stmt) return true;

   switch (ivl_statement_type(stmt)) {
   case IVL_ST_NOOP:
   case IVL_ST_NONE:
      return true;

   case IVL_ST_ASSIGN:
   case IVL_ST_ASSIGN_NB: {
      // Must be a simple assignment with no delay
      if (ivl_stmt_delay_expr(stmt))
         return false;
      unsigned nlvals = ivl_stmt_lvals(stmt);
      if (nlvals != 1)
         return false;
      ivl_lval_t lval = ivl_stmt_lval(stmt, 0);
      ivl_signal_t sig = ivl_lval_sig(lval);
      if (!sig)
         return false;
      // Only handle simple whole-signal assignments
      if (ivl_lval_part_off(lval) || ivl_lval_idx(lval))
         return false;
      ivl_expr_t rval = ivl_stmt_rval(stmt);
      if (!rval)
         return false;
      init_assign_t ia = { sig, rval };
      assigns.push_back(ia);
      return true;
   }

   case IVL_ST_BLOCK: {
      unsigned count = ivl_stmt_block_count(stmt);
      for (unsigned i = 0; i < count; i++) {
         if (!is_time_zero_only(ivl_stmt_block_stmt(stmt, i), assigns))
            return false;
      }
      return true;
   }

   default:
      // Any other statement type (delay, wait, loop, conditional, etc.)
      // means this initial persists beyond time zero
      return false;
   }
}

/*
 * Check to see if the process should have a name.
 */
static const char * get_process_name(ivl_process_t proc)
{
   const char* name = "";
   // Look for always @(...) begin : <name> to find the name
   if (ivl_process_type(proc) == IVL_PR_ALWAYS) {
      ivl_statement_t stmt = ivl_process_stmt(proc);
      if (ivl_statement_type(stmt) == IVL_ST_WAIT) {
	 stmt = ivl_stmt_sub_stmt(stmt);
	 if (ivl_statement_type(stmt) == IVL_ST_BLOCK) {
	    ivl_scope_t proc_scope = ivl_stmt_block_scope(stmt);
	    if (proc_scope) name = ivl_scope_basename(proc_scope);
	 }
      }
   }

   return name;
}

/*
 * Convert a Verilog process to VHDL and add it to the architecture
 * of the given entity.
 */
static int generate_vhdl_process(vhdl_entity *ent, ivl_process_t proc)
{
   set_active_entity(ent);

   // Create a new process and store it in the entity's
   // architecture. This needs to be done first or the
   // parent link won't be valid (and draw_stmt needs this
   // to add information to the architecture)
   vhdl_process *vhdl_proc = new vhdl_process(get_process_name(proc));
   ent->get_arch()->add_stmt(vhdl_proc);

   // If this is an initial process, push signal initialisation
   // into the declarations
   vhdl_proc->get_scope()->set_initializing
      (ivl_process_type(proc) == IVL_PR_INITIAL);

   ivl_statement_t stmt = ivl_process_stmt(proc);
   int rc = draw_stmt(vhdl_proc, vhdl_proc->get_container(), stmt);
   if (rc != 0)
      return rc;

   // Replace each blocking-target signal with a process-local variable
   // shadow so we can drop the `wait for 0 ns;` statements that would
   // otherwise commit intermediate values and drive delta-cycle livelock
   // on self-sensitive always-comb blocks.  Only applied to non-initial
   // processes (initial blocks have different semantics and are emitted
   // as deposit-style assignments anyway).
   if (ivl_process_type(proc) != IVL_PR_INITIAL)
      shadow_blocking_targets(vhdl_proc, ent);

   // Initial processes are translated to VHDL processes with
   // no sensitivity list and and indefinite wait statement at
   // the end
   // However, if no statements were added to the container
   // by draw_stmt, don't bother adding a wait as `emit'
   // will optimise the process out of the output
   // IVL_PR_FINAL (SystemVerilog `final`) also runs its body once and must
   // then suspend; without the trailing wait it becomes a free-running VHDL
   // process that infinite-loops at time 0 and deadlocks the simulation.
   bool is_initial = ivl_process_type(proc) == IVL_PR_INITIAL
                  || ivl_process_type(proc) == IVL_PR_FINAL;
   bool is_empty = vhdl_proc->get_container()->empty();

   if (is_initial && !is_empty) {
      vhdl_wait_stmt *wait = new vhdl_wait_stmt();
      vhdl_proc->get_container()->add_stmt(wait);
   }

   // An always-process whose body collapsed to only null statements
   // (e.g. `always @* q <= 1;` where iverilog elided the assignment
   // because the sensitivity list is empty and the RHS has no sources)
   // would loop forever in VHDL.  The Verilog intent is "this never
   // executes": add an unconditional wait so the process suspends
   // permanently after one entry.
   if (!is_initial && !is_empty) {
      stmt_container::stmt_list_t &stmts =
         vhdl_proc->get_container()->get_stmts();
      bool only_null = true;
      for (stmt_container::stmt_list_t::const_iterator it = stmts.begin();
           it != stmts.end(); ++it) {
         if (dynamic_cast<vhdl_null_stmt*>(*it) == NULL) {
            only_null = false;
            break;
         }
      }
      if (only_null)
         vhdl_proc->get_container()->add_stmt(new vhdl_wait_stmt());
   }

   // Add a comment indicating where it came from
   ivl_scope_t scope = ivl_process_scope(proc);
   const char *type = ivl_process_type(proc) == IVL_PR_INITIAL
      ? "initial" : "always";
   std::ostringstream ss;
   ss << "Generated from " << type << " process in "
      << ivl_scope_tname(scope) << " ("
      << ivl_process_file(proc) << ":"
      << ivl_process_lineno(proc) << ")";
   vhdl_proc->set_comment(ss.str());

   set_active_entity(NULL);
   return 0;
}

/*
 * Escape a string for use inside a VHDL string literal.
 * Doubles any embedded quote characters.
 */
/*
 * Generate a concurrent sv_analog() procedure call for an
 * analog process. The analog block body is reconstructed as
 * a Verilog-A string and passed as a string argument.
 */
static int generate_analog_call(vhdl_entity *ent, ivl_process_t proc,
                                ivl_scope_t scope)
{
   // Build metadata prefix: MODULE:<name>|PORT:<name>:<dir>:<disc>|...||<body>
   std::ostringstream meta;
   meta << "MODULE:" << ivl_scope_tname(scope);

   unsigned nsigs = ivl_scope_sigs(scope);
   for (unsigned i = 0; i < nsigs; i++) {
      ivl_signal_t sig = ivl_scope_sig(scope, i);
      ivl_signal_port_t pt = ivl_signal_port(sig);
      if (pt == IVL_SIP_NONE) continue;

      const char *dir = (pt == IVL_SIP_INPUT) ? "input"
                      : (pt == IVL_SIP_OUTPUT) ? "output" : "inout";
      const char *disc = "";
      ivl_discipline_t d = ivl_signal_discipline(sig);
      if (d) disc = ivl_discipline_name(d);

      meta << "|PORT:" << ivl_signal_basename(sig) << ":" << dir << ":" << disc;
   }
   meta << "||";

   // Build body from analog statement
   ivl_statement_t stmt = ivl_process_stmt(proc);
   std::string body = analog_stmt_to_str(stmt);

   // Combine metadata + body. Embedded-quote escaping is handled uniformly by
   // vhdl_const_string::emit, so pass the raw string here.
   std::string full = meta.str() + body;

   vhdl_conc_pcall_stmt *pcall = new vhdl_conc_pcall_stmt("sv_analog");
   pcall->add_expr(new vhdl_const_string(full));

   // Add source location comment
   std::ostringstream ss;
   ss << "Analog block from " << ivl_process_file(proc) << ":"
      << ivl_process_lineno(proc);
   pcall->set_comment(ss.str());

   ent->get_arch()->add_stmt(pcall);
   return 0;
}

extern "C" int draw_process(ivl_process_t proc, void *)
{
   ivl_scope_t scope = ivl_process_scope(proc);

   if (!is_default_scope_instance(scope))
      return 0;  // Ignore this process at it's not in a scope that
                 // we're using to generate code

   debug_msg("Translating process in scope type %s (%s:%d)",
             ivl_scope_tname(scope), ivl_process_file(proc),
             ivl_process_lineno(proc));

   // Record the process's own scope (before skipping up to the module) for
   // per-scope facts: %m wants the exact scope's hierarchical name (which may
   // be a generate/begin block, e.g. "main.genblk1"), while $time only needs
   // the timescale, which such scopes inherit from the module. See state.cc.
   set_active_scope(scope);

   // Skip over any generate and begin scopes until we find
   // the module that contains them - this is where we will
   // generate the process
   while (ivl_scope_type(scope) == IVL_SCT_GENERATE
      || ivl_scope_type(scope) == IVL_SCT_BEGIN)
      scope = ivl_scope_parent(scope);

   assert(ivl_scope_type(scope) == IVL_SCT_MODULE);
   vhdl_entity *ent = find_entity(scope);
   assert(ent != NULL);

   // Analog processes become concurrent sv_analog() calls
   if (ivl_process_analog(proc)) {
      if (!get_sv2vhdl_mode())
         return 0;  // Skip analog outside sv2vhdl mode
      return generate_analog_call(ent, proc, scope);
   }

   // For initial processes, extract any leading assignments that occur
   // before the first delay/wait/event and apply them as signal
   // declaration defaults. This avoids creating an extra VHDL driver
   // that would conflict with always processes driving the same
   // signals (causing 'U' due to resolution).
   //
   // The remaining statements (after the first wait) are still
   // generated as a process. If the initial only has prefix
   // assignments and nothing else, the process is suppressed entirely.
   if (ivl_process_type(proc) == IVL_PR_INITIAL) {
      ivl_statement_t stmt = ivl_process_stmt(proc);
      std::vector<init_assign_t> assigns;
      // Check for time-zero-only case first
      if (is_time_zero_only(stmt, assigns) && !assigns.empty()) {
         vhdl_scope *arch_scope = ent->get_arch()->get_scope();
         bool all_ok = true;
         for (auto &ia : assigns) {
            std::string name = make_safe_name(ia.sig);
            vhdl_decl *decl = arch_scope->get_decl(name);
            if (!decl) {
               all_ok = false;
               break;
            }
            if (!decl->has_initial()) {
               vhdl_expr *init = translate_expr(ia.value);
               if (init) {
                  decl->set_initial(init);
                  g_hoisted_signals.insert(ia.sig);
               }
            }
         }
         if (all_ok) {
            debug_msg("Converted time-zero initial to signal defaults (%s:%d)",
                      ivl_process_file(proc), ivl_process_lineno(proc));
            return 0;
         }
      }
      // For initial blocks that persist beyond time zero, we don't
      // extract prefix assignments — the process needs to remain as-is
      // since it creates drivers for signals it assigns later.
   }

   return generate_vhdl_process(ent, proc);
}
