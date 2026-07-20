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
#include <map>
#include <climits>
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

// Evaluate a compile-time-constant slice expression (int literals composed
// with +,-,*). Generate-scope flattening emits offsets like (0*2)+0 that
// must compare equal to a plain 0, or find_write_info calls two writes to
// the same bit "mixed slices" and the shadow pass takes the whole-signal
// path — fatal when several processes share slices of one signal (EH2
// icache bypass any_addr_match: 8 scope processes each republished stale
// snapshots of the other scopes' bits).
static bool const_value_of(vhdl_expr *e, int64_t &out)
{
   if (vhdl_const_int *ci = dynamic_cast<vhdl_const_int*>(e)) {
      out = ci->get_value();
      return true;
   }
   vhdl_binop_expr *b = dynamic_cast<vhdl_binop_expr*>(e);
   if (b == NULL)
      return false;
   int64_t acc = 0;
   bool first = true;
   for (std::list<vhdl_expr*>::const_iterator it = b->get_operands().begin();
        it != b->get_operands().end(); ++it) {
      int64_t v;
      if (!const_value_of(*it, v))
         return false;
      if (first) {
         acc = v;
         first = false;
         continue;
      }
      switch (b->get_op()) {
      case VHDL_BINOP_ADD:  acc += v; break;
      case VHDL_BINOP_SUB:  acc -= v; break;
      case VHDL_BINOP_MULT: acc *= v; break;
      default: return false;
      }
   }
   if (first)
      return false;
   out = acc;
   return true;
}

static bool same_const_int(vhdl_expr *a, vhdl_expr *b)
{
   int64_t va, vb;
   if (const_value_of(a, va) && const_value_of(b, vb))
      return va == vb;
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

// Rename ONLY the LHS base of nbassigns targeting `name`. The writes set
// from find_vars also contains refs inside LHS slice indices (a[adr] puts
// both `a` and `adr` there), and an index must keep reading the SIGNAL --
// renaming it to a shadow that updated earlier in the block would apply the
// post-assign index (Verilog evaluates NBA indices at statement time).
static void rename_nbassign_lhs(stmt_container *c, const std::string &name,
                                const std::string &var_name);

static void rename_nbassign_lhs_recurse(vhdl_seq_stmt *s,
                                        const std::string &name,
                                        const std::string &var_name)
{
   if (vhdl_nbassign_stmt *nb = dynamic_cast<vhdl_nbassign_stmt*>(s)) {
      if (nb->get_lhs()->get_name() == name)
         nb->get_lhs()->set_name(var_name);
   }
   std::vector<stmt_container*> subs;
   s->get_sub_containers(subs);
   for (stmt_container *sc : subs)
      rename_nbassign_lhs(sc, name, var_name);
}

static void rename_nbassign_lhs(stmt_container *c, const std::string &name,
                                const std::string &var_name)
{
   for (vhdl_seq_stmt *s : c->get_stmts())
      rename_nbassign_lhs_recurse(s, name, var_name);
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
   int64_t folded;
   if (const_value_of(e, folded))
      return new vhdl_const_int(folded);
   if (vhdl_const_int *ci = dynamic_cast<vhdl_const_int*>(e))
      return new vhdl_const_int(ci->get_value());
   return NULL;
}

// Verilog NBA-region commit semantics for edge-triggered processes (STD_MX).
// A clock-gate (ICG latch + AND) fires its edge one-or-more DELTAS after the
// raw clock edge; by then flops that committed at delta 0-1 have already
// propagated through combinational logic, so the gated flop captures
// POST-edge values (a race Verilog forbids: all nonblocking commits land in
// the NBA region at end of timestep). Restore that ordering in translation:
// evaluate every nonblocking RHS at statement time into a shadow variable,
// then commit all of them after `wait for 0 ns` -- which STD_MX routes to
// the INACTIVE region, i.e. after every same-timestep edge process (gated or
// not) has evaluated with the pre-edge snapshot.
static void nba_defer_commits(vhdl_process *vhdl_proc, vhdl_entity *ent)
{
   if (!get_sv2vhdl_mode())
      return;
   if (!vhdl_proc->is_edge_triggered() || vhdl_proc->contains_wait_stmt())
      return;

   stmt_container *body = vhdl_proc->get_container();
   vhdl_scope *proc_scope = vhdl_proc->get_scope();
   vhdl_scope *arch_scope = ent->get_arch()->get_scope();

   vhdl_var_set_t reads, writes;
   body->find_vars(reads, writes);

   // Collect candidate target signal names (deduped).
   std::set<std::string> targets;
   for (vhdl_var_set_t::iterator it = writes.begin(); it != writes.end(); ++it)
      targets.insert((*it)->get_name());

   std::list<vhdl_seq_stmt*> seeds;
   std::list<vhdl_seq_stmt*> commits;

   for (std::set<std::string>::const_iterator tit = targets.begin();
        tit != targets.end(); ++tit) {
      const std::string &sig_name = *tit;

      vhdl_decl *sig_decl = arch_scope->get_decl(sig_name);
      if (sig_decl == NULL)
         continue;
      if (sig_decl->assignment_type() != vhdl_decl::ASSIGN_NONBLOCK)
         continue;   // variables and blocking shadows commit immediately

      WriteInfo info;
      find_write_info(body, sig_name, info);
      if (!info.found)
         continue;   // no nbassign to this signal

      // A single static slice commits just that slice (keeps this process's
      // driver footprint unchanged). Mixed or dynamic slices fall back to a
      // WHOLE-signal shadow: seed the full value, write through the original
      // (possibly dynamic) index into the variable, commit the whole signal.
      // The dynamic index expression itself still reads SIGNALS (pre-edge,
      // Verilog statement-time evaluation). Whole commits assume this
      // process is the signal's only driver -- true for the memory/buffer
      // always blocks this matters for (the gated-clock fill buffers).
      vhdl_expr *commit_lhs_slice = NULL;
      vhdl_expr *commit_rhs_slice = NULL;
      if (!info.ambiguous && info.slice) {
         commit_lhs_slice = clone_slice(info.slice);
         commit_rhs_slice = clone_slice(info.slice);
         if (commit_lhs_slice == NULL || commit_rhs_slice == NULL) {
            delete commit_lhs_slice;
            delete commit_rhs_slice;
            commit_lhs_slice = commit_rhs_slice = NULL;  // whole-signal
         }
      }

      std::string var_name = "v_nba_" + sig_name;
      while (proc_scope->have_declared(var_name))
         var_name += "_";

      const vhdl_type *src_type = sig_decl->get_type();
      vhdl_var_decl *var_decl =
         new vhdl_var_decl(var_name, new vhdl_type(*src_type));
      proc_scope->add_decl(var_decl);

      // Rename ONLY the nbassign LHS bases to the shadow; reads AND slice
      // indices keep the signal (Verilog NBA evaluates both at statement
      // time against pre-edge values).
      rename_nbassign_lhs(body, sig_name, var_name);
      mark_var_assigns(body, var_name);

      // Seed each iteration so a conditionally-skipped write commits the
      // unchanged pre-edge value (a value-level no-op).
      {
         vhdl_var_ref *seed_lhs =
            new vhdl_var_ref(var_name, new vhdl_type(*src_type));
         vhdl_var_ref *seed_rhs =
            new vhdl_var_ref(sig_name, new vhdl_type(*src_type));
         seeds.push_back(new vhdl_assign_stmt(seed_lhs, seed_rhs));
      }

      // Commit (only the statically-written slice, to keep this process's
      // driver footprint unchanged).
      {
         vhdl_var_ref *lhs =
            new vhdl_var_ref(sig_name, new vhdl_type(*src_type));
         if (commit_lhs_slice)
            lhs->set_slice(commit_lhs_slice, info.slice_width);
         vhdl_var_ref *rhs =
            new vhdl_var_ref(var_name, new vhdl_type(*src_type));
         if (commit_rhs_slice)
            rhs->set_slice(commit_rhs_slice, info.slice_width);
         commits.push_back(new vhdl_nbassign_stmt(lhs, rhs));
      }
   }

   if (commits.empty())
      return;

   stmt_container::stmt_list_t &stmts = body->get_stmts();
   for (std::list<vhdl_seq_stmt*>::reverse_iterator it = seeds.rbegin();
        it != seeds.rend(); ++it)
      stmts.push_front(*it);

   // One hop to the Verilog NBA region, then all commits.
   stmts.push_back(new vhdl_wait_stmt(VHDL_WAIT_FOR0));
   for (std::list<vhdl_seq_stmt*>::iterator it = commits.begin();
        it != commits.end(); ++it)
      stmts.push_back(*it);

   // A process with a wait may not also have a sensitivity list: move the
   // clock/reset sensitivity into a trailing `wait on`.
   vhdl_wait_stmt *trailing = new vhdl_wait_stmt(VHDL_WAIT_ON);
   string_list_t &sens = vhdl_proc->get_sensitivity();
   for (string_list_t::const_iterator it = sens.begin();
        it != sens.end(); ++it)
      trailing->add_sensitivity(*it);
   sens.clear();
   stmts.push_back(trailing);
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
      if (!info.found)
         continue;

      // Mixed-slice or unclonable-slice writes can't get a slice-accurate
      // commit. Leaving them as plain `<=` is correct ONLY for write-only
      // patterns; when the process also READS the target, the emulated
      // blocking read-modify-write accumulates across activations (each
      // `<=` lands after the run, so reads see the previous activation —
      // the EH2 icache rd_mux `x = '0; for.. x |= ..` served stale rows
      // forever). Fall back to a WHOLE-SIGNAL shadow: seed v := sig,
      // rename every ref, commit sig <= v — sole-driver assumption, the
      // same contract as the NBA whole-signal fallback.
      bool whole_signal = false;
      vhdl_expr *commit_lhs_slice = NULL;
      vhdl_expr *commit_rhs_slice = NULL;
      if (info.ambiguous)
         whole_signal = true;
      else if (info.slice) {
         commit_lhs_slice = clone_slice(info.slice);
         commit_rhs_slice = clone_slice(info.slice);
         if (commit_lhs_slice == NULL || commit_rhs_slice == NULL) {
            delete commit_lhs_slice;
            delete commit_rhs_slice;
            commit_lhs_slice = commit_rhs_slice = NULL;
            whole_signal = true;
         }
      }
      if (whole_signal) {
         vhdl_var_set_t rmw_reads, rmw_writes;
         body->find_vars(rmw_reads, rmw_writes);
         bool reads_target = false;
         for (vhdl_var_set_t::iterator rit = rmw_reads.begin();
              rit != rmw_reads.end(); ++rit) {
            if ((*rit)->get_name() == sig_name) {
               reads_target = true;
               break;
            }
         }
         if (!reads_target)
            continue;   // write-only: plain `<=` slice writes are correct
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
   if (ivl_process_type(proc) != IVL_PR_INITIAL) {
      shadow_blocking_targets(vhdl_proc, ent);
      nba_defer_commits(vhdl_proc, ent);
   }

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
               // Only hoist when the value's type/width actually matches the
               // declaration -- a width-mismatched initial (e.g. a vector
               // constant against a scalar decl from a forward-typedef'd
               // signal) is a hard VHDL error at the declaration; leave those
               // assignments in the process where normal casting applies.
               if (init && init->get_type() && decl->get_type()
                   && init->get_type()->get_name()
                         == decl->get_type()->get_name()
                   && init->get_type()->get_width()
                         == decl->get_type()->get_width()) {
                  decl->set_initial(init);
                  g_hoisted_signals.insert(ia.sig);
               }
               else if (init) {
                  all_ok = false;   // keep the initial process for this one
                  break;
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

/*
 * Comb-cone fusion. The per-gate/per-LPM draws emit ONE PROCESS PER
 * INTERMEDIATE (`process (all) is begin tmp <= expr; end`), so logic
 * depth becomes delta-cycle count and every level re-wakes the whole
 * downstream cloud (measured on VeeR-EH2: ~130 proc-deltas and ~95K
 * activations per clock cycle, 81.8% of eval time in 1024+-wide deltas).
 *
 * Fuse all such single-assign waitless processes in an architecture
 * into ONE process, statements in topological order. Each member
 * becomes the pair
 *     v_T := <rhs with member-def reads renamed to their v_>;
 *     T    <= v_T;
 * The variable carries the fresh value within the single activation
 * (zero internal deltas); the signal assign still publishes exactly
 * one event per change for outside readers, and the driver census is
 * unchanged. The fused process gets an explicit sensitivity list of
 * EXTERNAL reads only, so it never wakes on its own outputs.
 * Multi-defined names and cycle members are left as-is (Kahn survivors
 * only). Kill-switch: SV2VHDL_NO_FUSE=1.
 */
void fuse_comb_processes(vhdl_entity *ent)
{
   vhdl_arch *arch = ent->get_arch();
   if (arch == NULL)
      return;

   // Candidates: sv2vhdl-mode concurrent assigns (each emits as its own
   // process(all)) — iverilog's per-operator netlist shrapnel.
   struct member_t {
      vhdl_cassign_stmt *ca;
      std::string def;
      member_t *dsu;         // union-find parent
      vhdl_var_ref *lhs() const { return ca->get_lhs(); }
      vhdl_expr *rhs() const { return ca->get_rhs(); }
   };
   std::list<member_t> cand;
   std::map<std::string, int> def_count;

   conc_stmt_list_t &items = arch->get_stmts();
   for (conc_stmt_list_t::iterator it = items.begin();
        it != items.end(); ++it) {
      vhdl_cassign_stmt *ca = dynamic_cast<vhdl_cassign_stmt*>(*it);
      if (ca == NULL || !ca->is_simple())
         continue;
      if (ca->get_lhs() == NULL || ca->get_rhs() == NULL)
         continue;
      if (ca->get_lhs()->get_slice() != NULL)   // part-writes stay put
         continue;
      if (ca->get_lhs()->get_type() == NULL)
         continue;
      member_t m = { ca, ca->get_lhs()->get_name(), NULL };
      cand.push_back(m);
      def_count[m.def]++;
   }
   for (std::list<member_t>::iterator it = cand.begin(); it != cand.end(); )
      it = (def_count[it->def] > 1) ? cand.erase(it) : ++it;

   // Drop self-loop members (T reads T) before graph building.
   std::map<std::string, member_t*> defs;
   for (std::list<member_t>::iterator it = cand.begin(); it != cand.end(); ++it)
      defs[it->def] = &*it;
   for (std::list<member_t>::iterator it = cand.begin(); it != cand.end(); ) {
      vhdl_var_set_t reads;
      it->rhs()->find_vars(reads);
      bool self = false;
      for (vhdl_var_set_t::iterator r = reads.begin(); r != reads.end(); ++r)
         if ((*r)->get_name() == it->def) { self = true; break; }
      if (self) {
         defs.erase(it->def);
         it = cand.erase(it);
      }
      else
         ++it;
   }
   if (cand.size() < 2)
      return;

   // Def/use edges among members; union-find groups CONNECTED dataflow
   // (a cone). Unrelated members stay in separate blocks so each block's
   // wake-set is only its own inputs — the whole-arch experiment showed
   // that merging unrelated members multiplies wasted re-execution.
   struct dsu {
      static member_t *find(member_t *m) {
         while (m->dsu != NULL) {
            if (m->dsu->dsu != NULL)
               m->dsu = m->dsu->dsu;
            m = m->dsu;
         }
         return m;
      }
      static void unite(member_t *a, member_t *b) {
         a = find(a); b = find(b);
         if (a != b)
            b->dsu = a;
      }
   };

   std::map<member_t*, unsigned> indeg;
   std::map<member_t*, std::list<member_t*> > out;
   for (std::list<member_t>::iterator it = cand.begin(); it != cand.end(); ++it) {
      vhdl_var_set_t reads;
      it->rhs()->find_vars(reads);
      unsigned d = 0;
      for (vhdl_var_set_t::iterator r = reads.begin(); r != reads.end(); ++r) {
         std::map<std::string, member_t*>::iterator dd =
            defs.find((*r)->get_name());
         if (dd != defs.end()) {
            out[dd->second].push_back(&*it);
            dsu::unite(dd->second, &*it);
            d++;
         }
      }
      indeg[&*it] = d;
   }

   // Global Kahn order; members left unordered are in cycles and keep
   // their original processes (their reads of fused defs still see
   // evented deposits, so mixing is safe).
   std::list<member_t*> order, queue;
   for (std::list<member_t>::iterator it = cand.begin(); it != cand.end(); ++it)
      if (indeg[&*it] == 0)
         queue.push_back(&*it);
   while (!queue.empty()) {
      member_t *m = queue.front();
      queue.pop_front();
      order.push_back(m);
      std::list<member_t*> &o = out[m];
      for (std::list<member_t*>::iterator sit = o.begin(); sit != o.end(); ++sit)
         if (--indeg[*sit] == 0)
            queue.push_back(*sit);
   }

   // Group the ordered members by DSU component, preserving topo order.
   std::map<member_t*, std::list<member_t*> > comps;
   for (std::list<member_t*>::iterator it = order.begin(); it != order.end(); ++it)
      comps[dsu::find(*it)].push_back(*it);

   vhdl_scope *ascope = arch->get_scope();
   std::set<vhdl_cassign_stmt*> fused;
   unsigned blkno = 0;

   for (std::map<member_t*, std::list<member_t*> >::iterator ci = comps.begin();
        ci != comps.end(); ++ci) {
      std::list<member_t*> &mem = ci->second;
      if (mem.size() < 2)
         continue;

      // External sensitivity: reads outside this component's defs that
      // are signals in the arch scope.
      std::set<std::string> cdefs, externals;
      for (std::list<member_t*>::iterator it = mem.begin(); it != mem.end(); ++it)
         cdefs.insert((*it)->def);
      for (std::list<member_t*>::iterator it = mem.begin(); it != mem.end(); ++it) {
         vhdl_var_set_t reads;
         (*it)->rhs()->find_vars(reads);
         for (vhdl_var_set_t::iterator r = reads.begin(); r != reads.end(); ++r) {
            const std::string rn = (*r)->get_name();
            if (cdefs.count(rn))
               continue;
            vhdl_decl *d = ascope->get_decl(rn);
            if (d != NULL && d->assignment_type() == vhdl_decl::ASSIGN_NONBLOCK)
               externals.insert(rn);
         }
      }
      if (externals.empty())
         continue;

      // One process per cone: members verbatim, `<=` becomes a 2040
      // blocking deposit `:=` — immediate same-run readback for the
      // downstream members, one event per real value change for outside
      // readers, and IEEE 1364 force semantics from the runtime deposit.
      char nbuf[32];
      snprintf(nbuf, sizeof(nbuf), "comb_fused_%u", blkno++);
      vhdl_process *fp = new vhdl_process(nbuf);
      for (std::set<std::string>::iterator it = externals.begin();
           it != externals.end(); ++it)
         fp->add_sensitivity(*it);
      for (std::list<member_t*>::iterator it = mem.begin(); it != mem.end(); ++it) {
         fp->get_container()->add_stmt(
            new vhdl_assign_stmt((*it)->lhs(), (*it)->rhs()));
         fused.insert((*it)->ca);
      }
      arch->add_stmt(fp);
   }

   if (fused.empty())
      return;
   for (conc_stmt_list_t::iterator it = items.begin(); it != items.end(); ) {
      vhdl_cassign_stmt *ca = dynamic_cast<vhdl_cassign_stmt*>(*it);
      it = (ca != NULL && fused.count(ca)) ? items.erase(it) : ++it;
   }
}
