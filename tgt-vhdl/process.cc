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
#include "state.hh"

#include <iostream>
#include <cassert>
#include <sstream>
#include <algorithm>
#include <vector>
#include <set>
#include <utility>

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

   // Initial processes are translated to VHDL processes with
   // no sensitivity list and and indefinite wait statement at
   // the end
   // However, if no statements were added to the container
   // by draw_stmt, don't bother adding a wait as `emit'
   // will optimise the process out of the output
   bool is_initial = ivl_process_type(proc) == IVL_PR_INITIAL;
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
static std::string vhdl_escape_string(const std::string &s)
{
   std::string result;
   result.reserve(s.size());
   for (size_t i = 0; i < s.size(); i++) {
      result += s[i];
      if (s[i] == '"')
         result += '"';
   }
   return result;
}

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

   // Combine metadata + body, escape for VHDL string
   std::string full = meta.str() + body;
   std::string escaped = vhdl_escape_string(full);

   vhdl_conc_pcall_stmt *pcall = new vhdl_conc_pcall_stmt("sv_analog");
   pcall->add_expr(new vhdl_const_string(escaped));

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
