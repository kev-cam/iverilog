/*
 *  VHDL code generation for statements.
 *
 *  Copyright (C) 2008-2025  Nick Gasson (nick@nickg.me.uk)
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
#include "state.hh"

#include <iostream>
#include <cstring>
#include <cassert>
#include <sstream>
#include <typeinfo>
#include <limits>
#include <set>
#include <algorithm>
#include <iomanip>

using namespace std;

static void emit_wait_for_0(vhdl_procedural *proc, stmt_container *container,
                            ivl_statement_t stmt, vhdl_expr *expr);

/*
 * VHDL has no real equivalent of Verilog's $finish task. The
 * current solution is to use `assert false ...' to terminate
 * the simulator. This isn't great, as the simulator will
 * return a failure exit code when in fact it completed
 * successfully.
 *
 * An alternative is to use the VHPI interface supported by
 * some VHDL simulators and implement the $finish functionality
 * in C. This function can be enabled with the flag
 * -puse-vhpi-finish=1.
 */
static int draw_stask_finish(vhdl_procedural *, stmt_container *container,
                             ivl_statement_t)
{
   const char *use_vhpi = ivl_design_flag(get_vhdl_design(), "use-vhpi-finish");
   if (strcmp(use_vhpi, "1") == 0) {
      //get_active_entity()->requires_package("work.Verilog_Support");
      container->add_stmt(new vhdl_pcall_stmt("work.Verilog_Support.Finish"));
   }
   else if (get_sv2vhdl_mode()) {
      container->add_stmt(new vhdl_pcall_stmt("std.env.finish"));
   }
   else {
      container->add_stmt(
         new vhdl_report_stmt(new vhdl_const_string("SIMULATION FINISHED"),
                              SEVERITY_FAILURE));
   }

   return 0;
}

static char parse_octal(const char *p)
{
   assert(*p && *(p+1) && *(p+2));
   assert(isdigit(*p) && isdigit(*(p+1)) && isdigit(*(p+2)));

   return (*p - '0') * 64
      + (*(p+1) - '0') * 8
      + (*(p+2) - '0') * 1;
}

// Generate VHDL report statements for Verilog $display/$write
static int draw_stask_display(vhdl_procedural *proc,
                              stmt_container *container,
                              ivl_statement_t stmt)
{
   vhdl_binop_expr *text = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
                                               vhdl_type::string());

   const int count = ivl_stmt_parm_count(stmt);
   int i = 0;
   while (i < count) {
      // $display may have an empty parameter, in which case
      // the expression will be null
      // The behaviour here seems to be to output a space
      ivl_expr_t net = ivl_stmt_parm(stmt, i++);
      if (net == NULL) {
         text->add_expr(new vhdl_const_string(" "));
         continue;
      }

      if (ivl_expr_type(net) == IVL_EX_STRING) {
         ostringstream ss;
         for (const char *p = ivl_expr_string(net); *p; p++) {
            if (*p == '\\') {
               // Octal escape
               char ch = parse_octal(p+1);
               if (ch == '\n') {
                  // Is there a better way of handling newlines?
                  // Maybe generate another report statement
               }
               else
                  ss << ch;
               p += 3;
            }
            else if (*p == '%' && *(++p) != '%') {
               // Flush the output string up to this point
               text->add_expr(new vhdl_const_string(ss.str()));
               ss.str("");

               // Skip over width for now
               while (isdigit(*p)) ++p;

               switch (*p) {
               case 'm':
                  // TODO: we can get the module name via attributes
                  cerr << "Warning: no VHDL translation for %m format code"
                       << endl;
                  break;
               case 'h': case 'H': case 'x': case 'X':
               case 'b': case 'B':
               case 'o': case 'O':
                  {
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i++);
                     assert(netp);

                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return 1;

                     emit_wait_for_0(proc, container, stmt, base);

                     // Pick the sv_display_pkg function for this format
                     const char *func;
                     switch (*p) {
                     case 'h': case 'H': case 'x': case 'X':
                        func = "sv_hstr"; break;
                     case 'b': case 'B':
                        func = "sv_bstr"; break;
                     default:
                        func = "sv_ostr"; break;
                     }

                     // Wrap in std_logic_vector() if needed
                     if (base->get_type()) {
                        vhdl_type_name_t tn = base->get_type()->get_name();
                        if ((tn == VHDL_TYPE_UNSIGNED || tn == VHDL_TYPE_SIGNED)
                            && !base->constant()) {
                           vhdl_fcall *conv = new vhdl_fcall("std_logic_vector",
                              vhdl_type::std_logic_vector(
                                 base->get_type()->get_msb(),
                                 base->get_type()->get_lsb()));
                           conv->add_expr(base);
                           base = conv;
                        } else if (tn == VHDL_TYPE_LOGIC3D_VECTOR
                                   && !base->constant()) {
                           // sv2vhdl mode: a logic3d_vector formatted with %x/
                           // %h/%b/%o must be reduced to its value bits, not
                           // printed via logic3d_vector'image (a "(2,3,..)"
                           // tuple). Convert logic3d_vector -> unsigned ->
                           // std_logic_vector so sv_hstr/sv_bstr/sv_ostr apply
                           // (mirrors the case-statement selector path).
                           int w = base->get_type()->get_width();
                           vhdl_fcall *u = new vhdl_fcall("l3d_to_unsigned",
                                                          vhdl_type::nunsigned(w));
                           u->add_expr(base);
                           vhdl_fcall *conv = new vhdl_fcall("std_logic_vector",
                              vhdl_type::std_logic_vector(
                                 base->get_type()->get_msb(),
                                 base->get_type()->get_lsb()));
                           conv->add_expr(u);
                           base = conv;
                        } else if (tn == VHDL_TYPE_STD_LOGIC_VECTOR
                                   && !base->constant()) {
                           // Already std_logic_vector: use directly
                        } else {
                           // Integer, single-bit, constant, etc.: fall back
                           text->add_expr(base->cast(text->get_type()));
                           break;
                        }
                     }

                     vhdl_fcall *f = new vhdl_fcall(func,
                                                    vhdl_type::string());
                     f->add_expr(base);
                     text->add_expr(f);
                  }
                  break;
               default:
                  {
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i++);
                     assert(netp);

                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return 1;

                     emit_wait_for_0(proc, container, stmt, base);

                     text->add_expr(base->cast(text->get_type()));
                  }
               }
            }
            else
               ss << *p;
         }

         // Emit any non-empty string data left in the buffer
         if (!ss.str().empty())
            text->add_expr(new vhdl_const_string(ss.str()));
      }
      else {
         vhdl_expr *base = translate_expr(net);
         if (NULL == base)
            return 1;

         emit_wait_for_0(proc, container, stmt, base);

         text->add_expr(base->cast(text->get_type()));
      }
   }

   if (count == 0)
      text->add_expr(new vhdl_const_string(""));

   container->add_stmt(new vhdl_report_stmt(text));
   return 0;
}

/*
 * `$set_val(arr, idx0, idx1, …, idxN, val)` emits `arr(idx0)(idx1)…(idxN) := val;`
 * (or `<= val;` for signals). Workaround for iverilog's restriction on chained
 * procedural part-selects on the LHS — sv-normalize rewrites
 * `arr[i][j] = X;` as `$set_val(arr, i, j, X);`.
 */
static int draw_stask_set_val(vhdl_procedural *proc,
                               stmt_container *container,
                               ivl_statement_t stmt)
{
   const int count = ivl_stmt_parm_count(stmt);
   if (count < 3) {
      cerr << "Error: $set_val requires at least 3 args (arr, idx, val)" << endl;
      return 1;
   }

   // First parameter is the array signal; extract the underlying signal.
   ivl_expr_t arr_expr = ivl_stmt_parm(stmt, 0);
   if (!arr_expr || ivl_expr_type(arr_expr) != IVL_EX_SIGNAL) {
      cerr << "Error: first arg to $set_val must be a signal" << endl;
      return 1;
   }
   ivl_signal_t sig = ivl_expr_signal(arr_expr);
   string signame(get_renamed_signal(sig));
   vhdl_decl *decl = proc->get_scope()->get_decl(signame);
   if (!decl) {
      cerr << "Error: $set_val could not resolve signal " << signame << endl;
      return 1;
   }

   const vhdl_type *ltype = new vhdl_type(*decl->get_type());
   vhdl_var_ref *lhs = new vhdl_var_ref(signame, ltype);

   // Indices are parms 1 .. count-2; value is parm count-1.
   // Iverilog flattens packed N-D arrays to a single dimension in VHDL, so
   // we compute a flat offset: idx0*W1*W2*…*Wn + idx1*W2*…*Wn + … + idxn.
   // Each Wk is the bit-width of the k-th packed dimension below the outer.
   vhdl_type integer(VHDL_TYPE_INTEGER);
   const int nidx = count - 2;
   const unsigned packed_dims = ivl_signal_packed_dimensions(sig);
   int lhs_slice_width = 1;   // width of the LHS slice (>1 if inner dims unindexed)
   if (nidx >= 1 && (unsigned)nidx <= packed_dims) {
      // Build per-index dimension widths: dim 0 is innermost, dim packed_dims-1
      // is outermost. For an outer index `i`, its weight is the product of
      // the inner-dimension sizes.
      std::vector<int> dim_size(packed_dims, 1);
      for (unsigned d = 0; d < packed_dims; d++) {
         int msb = ivl_signal_packed_msb(sig, d);
         int lsb = ivl_signal_packed_lsb(sig, d);
         dim_size[d] = (msb >= lsb ? msb - lsb : lsb - msb) + 1;
      }

      // iverilog: dim 0 is the OUTER (first-declared) packed dim, dim
      // packed_dims-1 is the innermost. For an outer index p (0=outermost),
      // its weight is the product of dim sizes for dims (p+1 .. packed_dims-1).
      vhdl_expr *flat = NULL;
      for (int p = 0; p < nidx; p++) {
         ivl_expr_t idx_e = ivl_stmt_parm(stmt, p + 1);
         vhdl_expr *idx = translate_expr(idx_e);
         if (!idx) return 1;
         idx = idx->cast(&integer);
         int weight = 1;
         for (int d = p + 1; d < (int)packed_dims; d++)
            weight *= dim_size[d];
         vhdl_expr *term = idx;
         if (weight != 1) {
            vhdl_expr *w = new vhdl_const_int(weight);
            term = new vhdl_binop_expr(idx, VHDL_BINOP_MULT, w,
                                        new vhdl_type(VHDL_TYPE_INTEGER));
         }
         if (flat == NULL)
            flat = term;
         else
            flat = new vhdl_binop_expr(flat, VHDL_BINOP_ADD, term,
                                       new vhdl_type(VHDL_TYPE_INTEGER));
      }
      // If only the outer dims were indexed (nidx < packed_dims), the LHS is
      // not a single bit but a slice spanning the un-indexed inner dims:
      // arr[i][j] on [..][..][W] -> arr(flat + W-1 downto flat). Width = the
      // product of the inner (un-indexed) dimension sizes; 1 when fully indexed.
      int inner_w = 1;
      for (int d = nidx; d < (int)packed_dims; d++)
         inner_w *= dim_size[d];
      lhs->set_slice(flat, inner_w - 1);
      lhs_slice_width = inner_w;
   }
   else {
      // Fallback: chained slice (works when iverilog kept the array shape).
      for (int p = 1; p < count - 1; p++) {
         ivl_expr_t idx_e = ivl_stmt_parm(stmt, p);
         vhdl_expr *idx = translate_expr(idx_e);
         if (!idx) return 1;
         idx = idx->cast(&integer);
         if (p == 1)
            lhs->set_slice(idx, 0);
         else
            lhs->add_extra_slice(idx);
      }
   }

   ivl_expr_t val_e = ivl_stmt_parm(stmt, count - 1);
   vhdl_expr *val = translate_expr(val_e);
   if (!val) return 1;

   // When the LHS is a multi-bit slice (inner dims left unindexed) but the
   // value is narrower (e.g. arr[i][j] = '0 -> a scalar L3D_0), widen the
   // value to the slice width — Verilog pads the MSBs with 0.
   if (lhs_slice_width > 1 && val->get_type() != NULL
       && val->get_type()->get_width() != lhs_slice_width)
      val = val->cast(vhdl_type::logic3d_vector(lhs_slice_width - 1, 0));

   vhdl_assign_stmt *assign = new vhdl_assign_stmt(lhs, val);
   container->add_stmt(assign);
   return 0;
}

/*
 * SystemVerilog queue methods, lowered by iverilog to system tasks named
 * "$ivl_queue_method$<method>". The queue signal is parm 0 (see scope.cc for
 * the bounded ring-buffer model: <q> array + <q>_head + <q>_tail).
 *   push_back(v): <q>(<q>_tail mod DEPTH) <= v;  <q>_tail <= <q>_tail + 1
 *   delete(0)/pop_front: <q>_head <= <q>_head + 1   (drop the front element)
 * push touches only tail and delete only head, so a same-cycle push+pop
 * needs no read-modify-write and composes correctly under NBA.
 */
static int draw_queue_method(vhdl_procedural *proc, stmt_container *container,
                             ivl_statement_t stmt)
{
   const int qdepth = 64;
   const char *dollar = strrchr(ivl_stmt_name(stmt), '$');
   const char *method = dollar ? dollar + 1 : ivl_stmt_name(stmt);

   ivl_expr_t qe = ivl_stmt_parm(stmt, 0);
   if (!qe || ivl_expr_type(qe) != IVL_EX_SIGNAL) {
      error("queue method %s: first arg must be a signal", method);
      return 1;
   }
   string q(get_renamed_signal(ivl_expr_signal(qe)));
   vhdl_decl *adecl = proc->get_scope()->get_decl(q);
   if (!adecl) {
      error("queue method %s: signal %s not declared", method, q.c_str());
      return 1;
   }

   if (strcmp(method, "push_back") == 0) {
      ivl_expr_t ve = ivl_stmt_parm(stmt, ivl_stmt_parm_count(stmt) - 1);
      vhdl_expr *val = translate_expr(ve);
      if (!val) return 1;

      // <q>(<q>_tail mod DEPTH) <= val
      vhdl_var_ref *aref =
         new vhdl_var_ref(q.c_str(), new vhdl_type(*adecl->get_type()));
      vhdl_expr *idx = new vhdl_binop_expr(
         new vhdl_var_ref((q + "_tail").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
         VHDL_BINOP_MOD, new vhdl_const_int(qdepth),
         new vhdl_type(VHDL_TYPE_INTEGER));
      aref->set_slice(idx);
      container->add_stmt(new vhdl_nbassign_stmt(aref, val));

      // <q>_tail <= <q>_tail + 1
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref((q + "_tail").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
         new vhdl_binop_expr(
            new vhdl_var_ref((q + "_tail").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
            VHDL_BINOP_ADD, new vhdl_const_int(1),
            new vhdl_type(VHDL_TYPE_INTEGER))));
      return 0;
   }
   else if (strcmp(method, "delete") == 0 || strcmp(method, "pop_front") == 0) {
      // <q>_head <= <q>_head + 1
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref((q + "_head").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
         new vhdl_binop_expr(
            new vhdl_var_ref((q + "_head").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
            VHDL_BINOP_ADD, new vhdl_const_int(1),
            new vhdl_type(VHDL_TYPE_INTEGER))));
      return 0;
   }

   error("queue method %s not supported", method);
   return 1;
}

/*
 * Generate VHDL for system tasks (like $display). Not all of
 * these are supported.
 */
static int draw_stask(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt)
{
   const char *name = ivl_stmt_name(stmt);

   if (strcmp(name, "$display") == 0)
      return draw_stask_display(proc, container, stmt);
   else if (strcmp(name, "$write") == 0)
      return draw_stask_display(proc, container, stmt);
   else if (strcmp(name, "$finish") == 0)
      return draw_stask_finish(proc, container, stmt);
   else if (strcmp(name, "$set_val") == 0)
      return draw_stask_set_val(proc, container, stmt);
   else if (strncmp(name, "$ivl_queue_method$", 18) == 0
            || strncmp(name, "$ivl_darray_method$", 19) == 0)
      return draw_queue_method(proc, container, stmt);
   else {
      vhdl_seq_stmt *result = new vhdl_null_stmt();
      ostringstream ss;
      ss << "Unsupported system task " << name << " omitted here ("
         << ivl_stmt_file(stmt) << ":" << ivl_stmt_lineno(stmt) << ")";
      result->set_comment(ss.str());
      container->add_stmt(result);
      cerr << "Warning: no VHDL translation for system task " << name << endl;
      return 0;
   }
}

/*
 * Generate VHDL for a block of Verilog statements. If this block
 * doesn't have its own scope then this function does nothing, other
 * than recursively translate the block's statements and add them
 * to the process. This is OK as the stmt_container class behaves
 * like a Verilog block.
 *
 * If this block has its own scope with local variables then these
 * are added to the process as local variables and the statements
 * are generated as above.
 */
static int draw_block(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt, bool is_last)
{
   ivl_scope_t block_scope = ivl_stmt_block_scope(stmt);
   if (block_scope) {
      int nsigs = ivl_scope_sigs(block_scope);
      for (int i = 0; i < nsigs; i++) {
         ivl_signal_t sig = ivl_scope_sig(block_scope, i);
         // Guard against re-entry: when a parent module elaborates this same
         // block via different paths (e.g. in iterated/loop unrolling), the
         // signal may already be remembered. remember_signal asserts on dups.
         if (!seen_signal_before(sig))
            remember_signal(sig, proc->get_scope());

         std::string safe_name = make_safe_name(sig);
         if (!proc->get_scope()->have_declared(safe_name)) {
            const vhdl_type* type = vhdl_type::type_for(ivl_signal_width(sig),
                                                        ivl_signal_signed(sig));
            proc->get_scope()->add_decl
               (new vhdl_var_decl(safe_name, type));
         }
      }
   }

   int count = ivl_stmt_block_count(stmt);
   for (int i = 0; i < count; i++) {
      ivl_statement_t stmt_i = ivl_stmt_block_stmt(stmt, i);
      if (draw_stmt(proc, container, stmt_i, is_last && i == count - 1) != 0)
         return 1;
   }
   return 0;
}

/*
 * A no-op statement. This corresponds to a `null' statement in VHDL.
 */
static int draw_noop(vhdl_procedural *, stmt_container *container,
                     ivl_statement_t)
{
   container->add_stmt(new vhdl_null_stmt());
   return 0;
}

static vhdl_var_ref *make_assign_lhs(ivl_lval_t lval, vhdl_scope *scope)
{
   ivl_signal_t sig = ivl_lval_sig(lval);
   if (!sig) {
      error("Only signals as lvals supported at the moment");
      return NULL;
   }

   vhdl_expr *base = NULL;
   ivl_expr_t e_off = ivl_lval_part_off(lval);
   if (NULL == e_off)
      e_off = ivl_lval_idx(lval);
   if (e_off) {
      if ((base = translate_expr(e_off)) == NULL)
         return NULL;

      vhdl_type integer(VHDL_TYPE_INTEGER);
      base = base->cast(&integer);
   }

   unsigned lval_width = ivl_lval_width(lval);

   string signame(get_renamed_signal(sig));
   vhdl_decl *decl = scope->get_decl(signame);
   assert(decl);

   // Verilog allows assignments to elements that are constant in VHDL:
   // function parameters, for example
   // To work around this we generate a local variable to shadow the
   // constant and assign to that
   if (decl->assignment_type() == vhdl_decl::ASSIGN_CONST) {
      const string shadow_name = signame + "_Shadow";
      vhdl_var_decl* shadow_decl =
         new vhdl_var_decl(shadow_name, decl->get_type());
      shadow_decl->set_initial
         (new vhdl_var_ref(signame, decl->get_type()));
      scope->add_decl(shadow_decl);

      // Make sure all future references to this signal use the
      // shadow variable
      rename_signal(sig, shadow_name);

      // ...and use this new variable as the assignment LHS
      decl = shadow_decl;
   }

   const vhdl_type *ltype = new vhdl_type(*decl->get_type());
   vhdl_var_ref *lval_ref = new vhdl_var_ref(decl->get_name(), ltype);
   if (base) {
      if (decl->get_type()->get_name() == VHDL_TYPE_ARRAY)
         lval_ref->set_slice(base, 0);
      else if (ivl_signal_width(sig) > 1)
         lval_ref->set_slice(base, lval_width - 1);
   }

   return lval_ref;
}

static bool assignment_lvals(ivl_statement_t stmt, vhdl_procedural *proc,
                             list<vhdl_var_ref*> &lvals)
{
   int nlvals = ivl_stmt_lvals(stmt);
   for (int i = 0; i < nlvals; i++) {
      ivl_lval_t lval = ivl_stmt_lval(stmt, i);
      vhdl_var_ref *lhs = make_assign_lhs(lval, proc->get_scope());
      if (NULL == lhs)
         return false;

      lvals.push_back(lhs);
   }

   return true;
}

/*
 * Generate the right sort of assignment statement for assigning
 * `lhs' to `rhs'.
 */
static vhdl_abstract_assign_stmt *
assign_for(vhdl_decl::assign_type_t atype, vhdl_var_ref *lhs, vhdl_expr *rhs)
{
   switch (atype) {
   case vhdl_decl::ASSIGN_BLOCK:
   case vhdl_decl::ASSIGN_CONST:
      return new vhdl_assign_stmt(lhs, rhs);
   case vhdl_decl::ASSIGN_NONBLOCK:
      return new vhdl_nbassign_stmt(lhs, rhs);
   }
   assert(false);
   return NULL;
}

/*
 * Check that this assignment type is valid within the context of `proc'.
 * For example, a <= assignment is not valid within a function.
 */
bool check_valid_assignment(vhdl_decl::assign_type_t atype, vhdl_procedural *proc,
                            ivl_statement_t stmt)
{
   if (atype == vhdl_decl::ASSIGN_NONBLOCK &&
       !proc->get_scope()->allow_signal_assignment()) {
      error("Unable to translate assignment at %s:%d\n"
            "  Translating this would require generating a non-blocking (<=)\n"
            "  assignment in a VHDL context where this is disallowed (e.g.\n"
            "  a function).", ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
      return false;
   }
   else
      return true;
}

// Generate a "wait for 0 ns" statement to emulate the behaviour of
// Verilog blocking assignment using VHDL signals. This is only generated
// if we read from the target of a blocking assignment in the same
// process (i.e. it is only generated when required, not for every
// blocking assignment). An example:
//
//   begin
//     x = 5;
//     if (x == 2)
//       y = 7;
//   end
//
// Becomes:
//
//   x <= 5;
//   wait for 0 ns;    -- Required to implement assignment semantics
//   if x = 2 then
//     y <= 7;         -- No need for wait here, not read
//   end if;
//
static void emit_wait_for_0(vhdl_procedural *proc,
                            stmt_container *container,
                            ivl_statement_t stmt,
                            vhdl_expr *expr)
{
   vhdl_var_set_t read;
   expr->find_vars(read);

   bool need_wait_for_0 = false;
   for (vhdl_var_set_t::const_iterator it = read.begin();
        it != read.end(); ++it) {
      if (proc->is_blocking_target(*it))
         need_wait_for_0 = true;
   }

   const stmt_container::stmt_list_t &stmts = container->get_stmts();
   bool last_was_wait =
      !stmts.empty() && dynamic_cast<vhdl_wait_stmt*>(stmts.back());

   if (need_wait_for_0 && !last_was_wait) {
      debug_msg("Generated wait-for-0 for %s:%d",
                ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));

      vhdl_seq_stmt *wait = new vhdl_wait_stmt(VHDL_WAIT_FOR0);

      ostringstream ss;
      ss << "Read target of blocking assignment ("
         << ivl_stmt_file(stmt)
         << ":" << ivl_stmt_lineno(stmt) << ")";
      wait->set_comment(ss.str());

      container->add_stmt(wait);
      proc->added_wait_stmt();
   }
}

// Generate an assignment of type T for the Verilog statement stmt.
// If a statement was generated then `assign_type' will contain the
// type of assignment that was generated; this should be initialised
// to some sensible default.
void make_assignment(vhdl_procedural *proc, stmt_container *container,
                     ivl_statement_t stmt, bool emul_blocking,
                     vhdl_decl::assign_type_t& assign_type)
{
   list<vhdl_var_ref*> lvals;
   if (!assignment_lvals(stmt, proc, lvals))
      return;

   vhdl_expr *rhs, *rhs2 = NULL;
   ivl_expr_t rval = ivl_stmt_rval(stmt);
   if (ivl_expr_type(rval) == IVL_EX_TERNARY) {
      rhs = translate_expr(ivl_expr_oper2(rval));
      rhs2 = translate_expr(ivl_expr_oper3(rval));
      if (rhs2 == NULL)
         return;
   }
   else
      rhs = translate_expr(rval);
   if (rhs == NULL)
      return;

   // Handle compressed assignments (+=, -=, etc.)
   // ivl_stmt_opcode returns the operator character, or 0 for normal assign
   // Only blocking assignments (IVL_ST_ASSIGN) can have compressed opcodes
   char comp_op = (ivl_statement_type(stmt) == IVL_ST_ASSIGN)
      ? ivl_stmt_opcode(stmt) : 0;
   if (comp_op && lvals.size() == 1) {
      vhdl_binop_t binop;
      switch (comp_op) {
      case '+': binop = VHDL_BINOP_ADD; break;
      case '-': binop = VHDL_BINOP_SUB; break;
      case '*': binop = VHDL_BINOP_MULT; break;
      case '/': binop = VHDL_BINOP_DIV; break;
      case '%': binop = VHDL_BINOP_MOD; break;
      case '&': binop = VHDL_BINOP_AND; break;
      case '|': binop = VHDL_BINOP_OR; break;
      case '^': binop = VHDL_BINOP_XOR; break;
      case 'l': binop = VHDL_BINOP_SL; break;
      case 'r': binop = VHDL_BINOP_SR; break;
      default:
         cerr << "Warning: unsupported compressed assignment op '"
              << comp_op << "'" << endl;
         binop = VHDL_BINOP_ADD;
      }
      // Build: lhs <op> rhs
      vhdl_var_ref *lhs_read =
         make_assign_lhs(ivl_stmt_lval(stmt, 0), proc->get_scope());
      rhs = new vhdl_binop_expr(lhs_read, binop, rhs,
                                new vhdl_type(*lhs_read->get_type()));
   }

   emit_wait_for_0(proc, container, stmt, rhs);
   if (rhs2)
      emit_wait_for_0(proc, container, stmt, rhs2);

   if (lvals.size() == 1) {
      vhdl_var_ref *lhs = lvals.front();
      rhs = rhs->cast(lhs->get_type());

      ivl_expr_t i_delay;
      vhdl_expr *after = NULL;
      if ((i_delay = ivl_stmt_delay_expr(stmt)) != NULL) {
         after = translate_time_expr(i_delay);
         if (after == NULL)
            return;

         emit_wait_for_0(proc, container, stmt, after);
      }

      // Find the declaration of the LHS so we know what type
      // of assignment statement to generate (is it a signal,
      // a variable, etc?)
      vhdl_decl *decl = proc->get_scope()->get_decl(lhs->get_name());
      assign_type = decl->assignment_type();

      if (assign_type == vhdl_decl::ASSIGN_NONBLOCK && emul_blocking)
          proc->add_blocking_target(lhs);

      // A small optimisation is to expand ternary RHSs into an
      // if statement (eliminates a function call and produces
      // more idiomatic code)
      if (ivl_expr_type(rval) == IVL_EX_TERNARY) {
         rhs2 = rhs2->cast(lhs->get_type());
         vhdl_var_ref *lhs2 =
            make_assign_lhs(ivl_stmt_lval(stmt, 0), proc->get_scope());

         vhdl_expr *test = translate_expr(ivl_expr_oper1(rval));
         if (NULL == test)
            return;

         emit_wait_for_0(proc, container, stmt, test);

         if (!check_valid_assignment(decl->assignment_type(), proc, stmt))
            return;

         vhdl_if_stmt *vhdif = new vhdl_if_stmt(test);

         // True part
         {
            vhdl_abstract_assign_stmt *a =
               assign_for(decl->assignment_type(), lhs, rhs);
            if (after)
               a->set_after(after);
            vhdif->get_then_container()->add_stmt(a);
         }

         // False part
         {
            vhdl_abstract_assign_stmt *a =
               assign_for(decl->assignment_type(), lhs2, rhs2);
            if (after)
               a->set_after(translate_time_expr(i_delay));
            vhdif->get_else_container()->add_stmt(a);
         }

         container->add_stmt(vhdif);
         return;
      }

      // In initial processes, use deposit (:=) instead of signal
      // assignment (<=) for signals. This avoids creating a VHDL driver
      // that would conflict with always processes driving the same
      // signal. Deposit writes the effective value without a driver,
      // matching Verilog's shared-driver reg semantics.
      // NVC --std=2040 supports := on signals (T_DEPOSIT).
      //
      // Exception: an NBA with an `after` delay (`a <= #2 1;`) needs the
      // signal-assignment semantics so the value change is scheduled,
      // not deposited immediately.  vhdl_assign_stmt has no `after`
      // form, so keep it as a non-blocking signal assignment.
      vhdl_decl::assign_type_t atype = decl->assignment_type();
      if (proc->get_scope()->initializing()
          && atype == vhdl_decl::ASSIGN_NONBLOCK
          && after == NULL)
         atype = vhdl_decl::ASSIGN_BLOCK;

      if (!check_valid_assignment(atype, proc, stmt))
         return;

      vhdl_abstract_assign_stmt *a = assign_for(atype, lhs, rhs);
      container->add_stmt(a);

      a->set_after(after);
   }
   else {
      // Multiple lvals are implemented by first assigning the complete
      // RHS to a temporary, and then assigning each lval in turn as
      // bit-selects of the temporary

      static int tmp_count = 0;
      ostringstream ss;
      ss << "Verilog_Assign_Tmp_" << tmp_count++;

      vhdl_decl* tmp_decl = new vhdl_var_decl(ss.str(), rhs->get_type());
      proc->get_scope()->add_decl(tmp_decl);

      container->add_stmt(new vhdl_assign_stmt(tmp_decl->make_ref(), rhs));

      list<vhdl_var_ref*>::iterator it;
      int width_so_far = 0;
      for (it = lvals.begin(); it != lvals.end(); ++it) {
         vhdl_var_ref *tmp_rhs = tmp_decl->make_ref();

         int lval_width = (*it)->get_type()->get_width();
         vhdl_expr *slice_base = new vhdl_const_int(width_so_far);
         tmp_rhs->set_slice(slice_base, lval_width - 1);

         ivl_expr_t i_delay;
         vhdl_expr *after = NULL;
         if ((i_delay = ivl_stmt_delay_expr(stmt)) != NULL) {
            after = translate_time_expr(i_delay);
            if (after == NULL)
               return;

            emit_wait_for_0(proc, container, stmt, after);
         }

         // Find the declaration of the LHS so we know what type
         // of assignment statement to generate (is it a signal,
         // a variable, etc?)
         const vhdl_decl *decl = proc->get_scope()->get_decl((*it)->get_name());
         assign_type = decl->assignment_type();

         if (!check_valid_assignment(decl->assignment_type(), proc, stmt))
            return;

         vhdl_abstract_assign_stmt *a =
            assign_for(decl->assignment_type(), *it, tmp_rhs);
         if (after)
            a->set_after(after);

         container->add_stmt(a);

         width_so_far += lval_width;

         if (assign_type == vhdl_decl::ASSIGN_NONBLOCK && emul_blocking)
            proc->add_blocking_target(*it);
      }
   }
}

/*
 * A non-blocking assignment inside a process. The semantics for
 * this are essentially the same as VHDL's non-blocking signal
 * assignment.
 */
static int draw_nbassign(vhdl_procedural *proc, stmt_container *container,
                         ivl_statement_t stmt)
{
   assert(proc->get_scope()->allow_signal_assignment());

   vhdl_decl::assign_type_t ignored;
   make_assignment(proc, container, stmt, false, ignored);

   return 0;
}

static int draw_assign(vhdl_procedural *proc, stmt_container *container,
                       ivl_statement_t stmt)
{
   // SystemVerilog queue clear: q = {}  ->  reset ring-buffer cursors.
   if (ivl_stmt_lvals(stmt) == 1) {
      ivl_lval_t lv0 = ivl_stmt_lval(stmt, 0);
      ivl_signal_t lsig = lv0 ? ivl_lval_sig(lv0) : 0;
      if (lsig && ivl_signal_data_type(lsig) == IVL_VT_QUEUE) {
         string q(get_renamed_signal(lsig));
         container->add_stmt(new vhdl_nbassign_stmt(
            new vhdl_var_ref((q + "_head").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
            new vhdl_const_int(0)));
         container->add_stmt(new vhdl_nbassign_stmt(
            new vhdl_var_ref((q + "_tail").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
            new vhdl_const_int(0)));
         return 0;
      }
   }

   vhdl_decl::assign_type_t assign_type = vhdl_decl::ASSIGN_NONBLOCK;
   bool emulate_blocking = proc->get_scope()->allow_signal_assignment();

   make_assignment(proc, container, stmt, emulate_blocking, assign_type);

   // $random(seed) advances its seed (passed by reference). The value assigned
   // above is sv_random(seed) (emitted by translate_sfunc_random); re-apply it
   // to the seed so the next call sees the advanced state. Emitted as a normal
   // assignment, so it is `:=` for a variable seed and `<=` for a signal seed --
   // no inout param needed, and it composes with any target lvalue.
   if (get_sv2vhdl_mode() && ivl_stmt_lvals(stmt) == 1) {
      ivl_expr_t rval = ivl_stmt_rval(stmt);
      if (rval && ivl_expr_type(rval) == IVL_EX_SFUNC
          && strcmp(ivl_expr_name(rval), "$random") == 0
          && ivl_expr_parms(rval) >= 1
          && ivl_expr_type(ivl_expr_parm(rval, 0)) == IVL_EX_SIGNAL) {
         ivl_signal_t ssig = ivl_expr_signal(ivl_expr_parm(rval, 0));
         string sname = get_renamed_signal(ssig);
         vhdl_decl *sdecl = proc->get_scope()->get_decl(sname);
         if (sdecl) {
            const vhdl_type *st = sdecl->get_type();
            vhdl_fcall *f = new vhdl_fcall("sv_random", new vhdl_type(*st));
            f->add_expr(new vhdl_var_ref(sname.c_str(), new vhdl_type(*st)));
            vhdl_var_ref *seed_lhs =
               new vhdl_var_ref(sname.c_str(), new vhdl_type(*st));
            // Mirror make_assignment: in an initial process, deposit (:=) into
            // a signal rather than a non-blocking <=, so the advance is visible
            // to the next read in the same process (and avoids a driver).
            vhdl_decl::assign_type_t satype = sdecl->assignment_type();
            if (proc->get_scope()->initializing()
                && satype == vhdl_decl::ASSIGN_NONBLOCK)
               satype = vhdl_decl::ASSIGN_BLOCK;
            container->add_stmt(assign_for(satype, seed_lhs, f));
         }
      }
   }

   return 0;
}

/*
 * Delay statements are equivalent to the `wait for' form of the
 * VHDL wait statement.
 */
static int draw_delay(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt)
{
   // This currently ignores the time units and precision
   // of the enclosing scope
   // A neat way to do this would be to make these values
   // constants in the scope (type is Time), and have the
   // VHDL wait statement compute the value from that.
   // The other solution is to add them as parameters to
   // the vhdl_process class
   vhdl_expr *time;
   if (ivl_statement_type(stmt) == IVL_ST_DELAY) {
      uint64_t value = ivl_stmt_delay_val(stmt);
      time = scale_time(get_active_entity(), value);
   }
   else {
      time = translate_time_expr(ivl_stmt_delay_expr(stmt));
      if (NULL == time)
         return 1;
   }

   ivl_statement_t sub_stmt = ivl_stmt_sub_stmt(stmt);
   vhdl_wait_stmt *wait =
      new vhdl_wait_stmt(VHDL_WAIT_FOR, time);

   // Remember that we needed a wait statement so if this is
   // a process it cannot have a sensitivity list
   proc->added_wait_stmt();

   container->add_stmt(wait);

   // Expand the sub-statement as well
   // Often this would result in a useless `null' statement which
   // is caught here instead
   if (ivl_statement_type(sub_stmt) != IVL_ST_NOOP)
      draw_stmt(proc, container, sub_stmt);

   // Any further assignments occur after simulation time 0
   // so they cannot be used to initialise signal declarations
   // (if this scope is an initial process)
   proc->get_scope()->set_initializing(false);

   return 0;
}

/*
 * Build a set of all the nexuses referenced by signals in `expr'.
 */
static void get_nexuses_from_expr(ivl_expr_t expr, set<ivl_nexus_t> &out)
{
   switch (ivl_expr_type(expr)) {
   case IVL_EX_SIGNAL:
      out.insert(ivl_signal_nex(ivl_expr_signal(expr), 0));
      break;
   case IVL_EX_TERNARY:
      get_nexuses_from_expr(ivl_expr_oper3(expr), out);
      // fallthrough
   case IVL_EX_BINARY:
      get_nexuses_from_expr(ivl_expr_oper2(expr), out);
      // fallthrough
   case IVL_EX_UNARY:
      get_nexuses_from_expr(ivl_expr_oper1(expr), out);
      break;
   default:
      break;
   }
}

/*
 * Attempt to identify common forms of wait statements and produce
 * more idiomatic VHDL than would be produced by the generic
 * draw_wait function. The main application of this is a input to
 * synthesis tools that don't synthesise the full VHDL language.
 * If none of these patterns are matched, the function returns false
 * and the default draw_wait is used.
 *
 * Current patterns:
 *   always @(posedge A or posedge B)
 *     if (A)
 *        ...
 *     else
 *        ...
 *
 *   This is assumed to be the template for a FF with asynchronous
 *   reset. A is assumed to be the reset as it is dominant. This will
 *   produce the following VHDL:
 *
 *   process (A, B) is
 *   begin
 *     if A = '1' then
 *       ...
 *     else if rising_edge(B) then
 *       ...
 *     end if;
 *   end process;
 */
static bool draw_synthesisable_wait(vhdl_process *proc, stmt_container *container,
                                    ivl_statement_t stmt)
{
   // At the moment this only detects FFs with an asynchronous reset
   // All other code will fall back on the default draw_wait

   // Store a set of the edge triggered signals
   // The second item is true if this is positive-edge
   set<ivl_nexus_t> edge_triggered;

   const int nevents = ivl_stmt_nevent(stmt);

   for (int i = 0; i < nevents; i++) {
      ivl_event_t event = ivl_stmt_events(stmt, i);

      if (ivl_event_nany(event) > 0)
         return false;

      int npos = ivl_event_npos(event);
      for (int j = 0; j < npos; j++)
         edge_triggered.insert(ivl_event_pos(event, j));

      int nneg = ivl_event_nneg(event);
      for (int j = 0; j < nneg; j++)
         edge_triggered.insert(ivl_event_neg(event, j));
   }

   // If we're edge-sensitive to less than two signals this doesn't
   // match the expected template, so use the default draw_wait
   if (edge_triggered.size() < 2)
      return false;

   // Now check to see if the immediately embedded statement is an `if'
   ivl_statement_t sub_stmt = ivl_stmt_sub_stmt(stmt);
   if (ivl_statement_type(sub_stmt) != IVL_ST_CONDIT)
      return false;

   // The if should have two branches: one is the reset branch and
   // one is the clocked branch
   if (ivl_stmt_cond_false(sub_stmt) == NULL)
      return false;

   // Check the first branch of the if statement
   // If it matches exactly one of the edge-triggered signals then assume
   // this is the (dominant) reset branch
   set<ivl_nexus_t> test_nexuses;
   get_nexuses_from_expr(ivl_stmt_cond_expr(sub_stmt), test_nexuses);

   // If the test is not a simple function of one variable then this
   // template will not work
   if (test_nexuses.size() != 1)
      return false;

   // Now subtracting this set from the set of edge triggered events
   // should leave just one nexus, which is hopefully the clock.
   // If not, then we fall back on the default draw_wait
   set<ivl_nexus_t> clock_net;
   set_difference(edge_triggered.begin(), edge_triggered.end(),
                  test_nexuses.begin(), test_nexuses.end(),
                  inserter(clock_net, clock_net.begin()));

   if (clock_net.size() != 1)
      return false;

   // Build a VHDL `if' statement to model this
   vhdl_expr *reset_test = translate_expr(ivl_stmt_cond_expr(sub_stmt));
   vhdl_if_stmt *body = new vhdl_if_stmt(reset_test);

   // Draw the reset branch
   draw_stmt(proc, body->get_then_container(), ivl_stmt_cond_true(sub_stmt));

   // Build a test for the clock event
   vhdl_fcall *edge = NULL;
   ivl_nexus_t the_clock_net = *clock_net.begin();
   for (int i = 0; i < nevents; i++) {
      ivl_event_t event = ivl_stmt_events(stmt, i);

      const unsigned npos = ivl_event_npos(event);
      for (unsigned j = 0; j < npos; j++) {
         if (ivl_event_pos(event, j) == the_clock_net)
            edge = new vhdl_fcall("rising_edge", vhdl_type::boolean());
      }

      const unsigned nneg = ivl_event_nneg(event);
      for (unsigned j = 0; j < nneg; j++)
         if (ivl_event_neg(event, j) == the_clock_net)
            edge = new vhdl_fcall("falling_edge", vhdl_type::boolean());
   }
   assert(edge);

   edge->add_expr(nexus_to_var_ref(proc->get_scope(), *clock_net.begin()));

   // Draw the clocked branch
   // For an asynchronous reset we just want this around the else branch,
   stmt_container *else_container = body->add_elsif(edge);

   draw_stmt(proc, else_container, ivl_stmt_cond_false(sub_stmt));

   if (proc->contains_wait_stmt()) {
      // Expanding the body produced a `wait' statement which can't
      // be included in a sensitised process so undo all this work
      // and fall back on the default draw_wait
      delete body;
      return false;
   }
   else
      container->add_stmt(body);

   // Add all the edge triggered signals to the sensitivity list
   for (set<ivl_nexus_t>::const_iterator it = edge_triggered.begin();
        it != edge_triggered.end(); ++it) {
      // Get the signal that represents this nexus in this scope
      vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), *it);

      proc->add_sensitivity(ref->get_name());

      // Don't need the reference any more
      delete ref;
   }

   // Don't bother with the default draw_wait
   return true;
}

/*
 * A wait statement waits for a level change on a @(..) list of
 * signals. The logic here might seem a little bit convoluted,
 * it attempts to always produce something that will simulate
 * correctly, and tries to produce something that will also
 * synthesise correctly (although not at the expense of simulation
 * accuracy).
 *
 * The difficulty stems from VHDL's restriction that a process with
 * a sensitivity list may not contain any `wait' statements: we need
 * to generate these to accurately model some Verilog statements.
 *
 * The steps followed are:
 *  1) Determine whether this is the top-level statement in the process
 *  2) If this is top-level, call draw_synthesisable_wait to see if the
 *     process and wait statement match any templates for which we know
 *     how to produce good, idiomatic synthesisable VHDL (e.g. FF with
 *     async reset)
 *  3) Determine whether the process is combinatorial (purely level
 *     sensitive), or sequential (edge sensitive)
 *  4) Draw all of the statements in the body
 *  5) One of the following will be true:
 *     A) The process is combinatorial, top-level, and there are
 *        no `wait' statements in the body: add all the level-sensitive
 *        signals to the VHDL sensitivity list
 *     B) The process is combinatorial, and there *are* `wait'
 *        statements in the body or it is not top-level: generate
 *        a VHDL `wait-on' statement at the end of the body containing
 *        the level-sensitive signals
 *     C) The process is sequential, top-level, and there are
 *        no `wait' statements in the body: build an `if' statement
 *        with the edge-detecting expression and wrap the process
 *        in it.
 *     D) The process is sequential, there *are* `wait' statements
 *        in the body, or it is not top-level: generate a VHDL
 *        `wait-until' with the edge-detecting expression and add
 *        it before the body of the wait event.
 */
static int draw_wait(vhdl_procedural *_proc, stmt_container *container,
                     ivl_statement_t stmt)
{
   // Wait statements only occur in processes
   vhdl_process *proc = dynamic_cast<vhdl_process*>(_proc);
   assert(proc);   // Catch not process

   // If this container is the top-level statement (i.e. it is the
   // first thing inside a process) then we can extract these
   // events out into the sensitivity list as long as we haven't
   // promoted any preceding assignments to initialisers
   bool is_top_level =
      container == proc->get_container()
      && container->empty()
      && !proc->get_scope()->hoisted_initialiser();

   // See if this can be implemented in a more idiomatic way before we
   // fall back on the generic translation
   if (is_top_level && draw_synthesisable_wait(proc, container, stmt))
      return 0;

   int nevents = ivl_stmt_nevent(stmt);

   bool combinatorial = true;  // True if no negedge/posedge events
   for (int i = 0; i < nevents; i++) {
      ivl_event_t event = ivl_stmt_events(stmt, i);
      if (ivl_event_npos(event) > 0 || ivl_event_nneg(event) > 0)
         combinatorial = false;
   }

   if (combinatorial) {
      // If the process has no wait statement in its body then
      // add all the events to the sensitivity list, otherwise
      // build a wait-on statement at the end of the process

      draw_stmt(proc, container, ivl_stmt_sub_stmt(stmt), true);

      vhdl_wait_stmt *wait = NULL;
      if (proc->contains_wait_stmt() || !is_top_level)
         wait = new vhdl_wait_stmt(VHDL_WAIT_ON);

      for (int i = 0; i < nevents; i++) {
         ivl_event_t event = ivl_stmt_events(stmt, i);

         int nany = ivl_event_nany(event);
         for (int j = 0; j < nany; j++) {
            ivl_nexus_t nexus = ivl_event_any(event, j);
            vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), nexus);

            if (wait)
               wait->add_sensitivity(ref->get_name());
            else
               proc->add_sensitivity(ref->get_name());
            delete ref;
         }
      }

      if (wait)
         container->add_stmt(wait);
   }
   else {
      // Build a test expression to represent the edge event
      // If this process contains no `wait' statements and this
      // is the top-level container, then we
      // wrap it in an `if' statement with this test and add the
      // edge triggered signals to the sensitivity, otherwise
      // build a `wait until' statement at the top of the process
      vhdl_binop_expr *test =
         new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());

      stmt_container tmp_container;
      draw_stmt(proc, &tmp_container, ivl_stmt_sub_stmt(stmt), true);

      for (int i = 0; i < nevents; i++) {
         ivl_event_t event = ivl_stmt_events(stmt, i);

         int nany = ivl_event_nany(event);
         for (int j = 0; j < nany; j++) {
            ivl_nexus_t nexus = ivl_event_any(event, j);
            vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), nexus);

            ref->set_name(ref->get_name() + "'Event");
            test->add_expr(ref);

            if (!proc->contains_wait_stmt() && is_top_level)
               proc->add_sensitivity(ref->get_name());
         }

         int nneg = ivl_event_nneg(event);
         for (int j = 0; j < nneg; j++) {
            ivl_nexus_t nexus = ivl_event_neg(event, j);
            vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), nexus);
            vhdl_fcall *detect =
               new vhdl_fcall("falling_edge", vhdl_type::boolean());
            detect->add_expr(ref);

            test->add_expr(detect);

            if (!proc->contains_wait_stmt() && is_top_level)
               proc->add_sensitivity(ref->get_name());
         }

         int npos = ivl_event_npos(event);
         for (int j = 0; j < npos; j++) {
            ivl_nexus_t nexus = ivl_event_pos(event, j);
            vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), nexus);
            vhdl_fcall *detect =
               new vhdl_fcall("rising_edge", vhdl_type::boolean());
            detect->add_expr(ref);

            test->add_expr(detect);

            if (!proc->contains_wait_stmt() && is_top_level)
               proc->add_sensitivity(ref->get_name());
         }
      }

      if (proc->contains_wait_stmt() || !is_top_level) {
         container->add_stmt(new vhdl_wait_stmt(VHDL_WAIT_UNTIL, test));
         container->move_stmts_from(&tmp_container);
      }
      else {
         // Wrap the whole process body in an `if' statement to detect
         // the edge event
         vhdl_if_stmt *edge_detect = new vhdl_if_stmt(test);

         // Move all the statements from the process body into the `if'
         // statement
         edge_detect->get_then_container()->move_stmts_from(&tmp_container);

         container->add_stmt(edge_detect);
      }

   }

   return 0;
}

static int draw_if(vhdl_procedural *proc, stmt_container *container,
                   ivl_statement_t stmt, bool is_last)
{
   vhdl_expr *test = translate_expr(ivl_stmt_cond_expr(stmt));
   if (NULL == test)
      return 1;

   emit_wait_for_0(proc, container, stmt, test);

   vhdl_if_stmt *vhdif = new vhdl_if_stmt(test);
   container->add_stmt(vhdif);

   ivl_statement_t cond_true_stmt = ivl_stmt_cond_true(stmt);
   if (cond_true_stmt)
      draw_stmt(proc, vhdif->get_then_container(), cond_true_stmt, is_last);

   ivl_statement_t cond_false_stmt = ivl_stmt_cond_false(stmt);
   if (cond_false_stmt)
      draw_stmt(proc, vhdif->get_else_container(), cond_false_stmt, is_last);

   return 0;
}

static vhdl_var_ref *draw_case_test(vhdl_procedural *proc, stmt_container *container,
                                    ivl_statement_t stmt)
{
   vhdl_expr *test = translate_expr(ivl_stmt_cond_expr(stmt));
   if (NULL == test)
      return NULL;

   // In sv2vhdl mode, extract the unsigned value field for case matching
   if (get_sv2vhdl_mode() && test->get_type()
       && test->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      int width = test->get_type()->get_width();
      vhdl_fcall *conv = new vhdl_fcall("l3d_to_unsigned",
                                         vhdl_type::nunsigned(width));
      conv->add_expr(test);
      test = conv;
   }

   // VHDL case expressions are required to be quite simple: variable
   // references or slices. So we may need to create a temporary
   // variable to hold the result of the expression evaluation
   if (typeid(*test) != typeid(vhdl_var_ref)) {
      // Find a unique name for the case expression variable.
      // Nested cases may have different widths, so we need separate variables.
      const vhdl_type *test_type = new vhdl_type(*test->get_type());
      std::string tmp_name_str = "Verilog_Case_Ex";
      int suffix = 0;
      while (proc->get_scope()->have_declared(tmp_name_str)
             && proc->get_scope()->get_decl(tmp_name_str)->get_type()->get_width()
                != test_type->get_width()) {
         tmp_name_str = "Verilog_Case_Ex_" + std::to_string(++suffix);
      }

      if (!proc->get_scope()->have_declared(tmp_name_str)) {
         proc->get_scope()->add_decl
            (new vhdl_var_decl(tmp_name_str, new vhdl_type(*test_type)));
      }

      vhdl_var_ref *tmp_ref = new vhdl_var_ref(tmp_name_str.c_str(), NULL);
      container->add_stmt(new vhdl_assign_stmt(tmp_ref, test));

      return new vhdl_var_ref(tmp_name_str.c_str(), test_type);
   }
   else
      return dynamic_cast<vhdl_var_ref*>(test);
}

// Return true if every case-choice expression in `stmt` is a constant.
// Verilog allows non-constant case choices (`case (1'b1) when sig:`) but
// VHDL requires choices to be locally static; if any branch is dynamic
// we'll have to emit an if/elsif chain instead.
static bool case_choices_all_static(ivl_statement_t stmt)
{
   int nbranches = ivl_stmt_case_count(stmt);
   for (int i = 0; i < nbranches; i++) {
      ivl_expr_t net = ivl_stmt_case_expr(stmt, i);
      if (!net) continue;   // default branch
      ivl_expr_type_t et = ivl_expr_type(net);
      if (et != IVL_EX_NUMBER && et != IVL_EX_STRING && et != IVL_EX_ULONG)
         return false;
   }
   return true;
}

static int draw_case(vhdl_procedural *proc, stmt_container *container,
                     ivl_statement_t stmt, bool is_last)
{
   vhdl_var_ref *test = draw_case_test(proc, container, stmt);
   if (NULL == test)
      return 1;

   if (!case_choices_all_static(stmt)) {
      // Emit an if/elsif chain: `when expr =>` becomes
      // `if test = expr then ... elsif test = expr2 then ...`.
      vhdl_if_stmt *if_chain = NULL;
      ivl_statement_t default_stmt = NULL;
      int nbranches = ivl_stmt_case_count(stmt);
      for (int i = 0; i < nbranches; i++) {
         ivl_expr_t net = ivl_stmt_case_expr(stmt, i);
         ivl_statement_t bstmt = ivl_stmt_case_stmt(stmt, i);
         if (!net) { default_stmt = bstmt; continue; }

         vhdl_expr *when = translate_expr(net);
         if (!when) return 1;
         when = when->cast(test->get_type());
         if (!when) return 1;

         vhdl_expr *cmp = new vhdl_binop_expr(
            new vhdl_var_ref(test->get_name().c_str(), test->get_type()),
            VHDL_BINOP_EQ, when, vhdl_type::boolean());

         stmt_container *body;
         if (if_chain == NULL) {
            if_chain = new vhdl_if_stmt(cmp);
            body = if_chain->get_then_container();
         } else {
            body = if_chain->add_elsif(cmp);
         }
         draw_stmt(proc, body, bstmt, is_last);
      }
      if (if_chain == NULL) {
         // Only had a default; emit it directly.
         if (default_stmt)
            draw_stmt(proc, container, default_stmt, is_last);
         return 0;
      }
      if (default_stmt)
         draw_stmt(proc, if_chain->get_else_container(), default_stmt, is_last);
      container->add_stmt(if_chain);
      return 0;
   }

   vhdl_case_stmt *vhdlcase = new vhdl_case_stmt(test);
   container->add_stmt(vhdlcase);

   // VHDL is more strict than Verilog about covering every
   // possible case. So make sure we add an 'others' branch
   // if there isn't a default one.
   bool have_others = false;

   int nbranches = ivl_stmt_case_count(stmt);
   for (int i = 0; i < nbranches; i++) {
      vhdl_expr *when;
      ivl_expr_t net = ivl_stmt_case_expr(stmt, i);
      if (net) {
         when = translate_expr(net)->cast(test->get_type());
         if (NULL == when)
            return 1;
      }
      else {
         when = new vhdl_var_ref("others", NULL);
         have_others = true;
      }

      vhdl_case_branch *branch = new vhdl_case_branch(when);
      vhdlcase->add_branch(branch);

      ivl_statement_t stmt_i = ivl_stmt_case_stmt(stmt, i);
      draw_stmt(proc, branch->get_container(), stmt_i, is_last);
   }

   if (!have_others) {
      vhdl_case_branch *others =
         new vhdl_case_branch(new vhdl_var_ref("others", NULL));
      others->get_container()->add_stmt(new vhdl_null_stmt());
      vhdlcase->add_branch(others);
   }

   return 0;
}

/*
 * Check to see if the given number (expression) can be represented
 * accurately in a long value.
 */
static bool number_is_long(ivl_expr_t expr)
{
   ivl_expr_type_t type = ivl_expr_type(expr);

   assert(type == IVL_EX_NUMBER || type == IVL_EX_ULONG);

   // Make sure the ULONG can be represented correctly in a long.
   if (type == IVL_EX_ULONG) {
      unsigned long val = ivl_expr_uvalue(expr);
      if (val > static_cast<unsigned>(numeric_limits<long>::max())) {
         return false;
      }
      return true;
   }

   // Check to see if the number actually fits in a long.
   unsigned nbits = ivl_expr_width(expr);
   if (nbits >= 8*sizeof(long)) {
      const char*bits = ivl_expr_bits(expr);
      char pad_bit = bits[nbits-1];
      for (unsigned idx = 8*sizeof(long); idx < nbits; idx++) {
         if (bits[idx] != pad_bit) return false;
      }
   }

   return true;
}

/*
 * Return the given number (expression) as a signed long value.
 *
 * Make sure to call number_is_long() first to verify that the number
 * can be represented accurately in a long value.
 */
static long get_number_as_long(ivl_expr_t expr)
{
   long imm = 0;
   switch (ivl_expr_type(expr)) {
   case IVL_EX_ULONG:
      imm = ivl_expr_uvalue(expr);
      break;

   case IVL_EX_NUMBER: {
      const char*bits = ivl_expr_bits(expr);
      unsigned nbits = ivl_expr_width(expr);
      if (nbits > 8*sizeof(long)) nbits = 8*sizeof(long);
      for (unsigned idx = 0; idx < nbits; idx++) {
         switch (bits[idx]) {
         case '0':
            break;
         case '1':
            imm |= 1L << idx;
            break;
         default:
            assert(0);
         }

         if (ivl_expr_signed(expr) && bits[nbits-1] == '1' &&
             nbits < 8*sizeof(long)) imm |= -1UL << nbits;
      }
      break;
   }

   default:
      assert(0);
   }
   return imm;
}

/*
 * Build a check against a constant 'x'. This is for an out of range
 * or undefined select.
 */
static void check_against_x(vhdl_binop_expr *all, const vhdl_var_ref *test,
                            ivl_expr_t expr, unsigned width, unsigned base,
                            bool is_casez)
{
   if (is_casez) {
      // For a casez we need to check against 'x'.
      for (unsigned i = 0; i < ivl_expr_width(expr); i++) {
         vhdl_binop_expr *sub_expr =
            new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
         const vhdl_type *type;
         vhdl_var_ref *ref;

         // Check if the test bit is 'z'.
         type = vhdl_type::nunsigned(width);
         ref = new vhdl_var_ref(test->get_name().c_str(), type);
         ref->set_slice(new vhdl_const_int(i+base));
         vhdl_binop_expr *cmp =
            new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
         cmp->add_expr(ref);
         cmp->add_expr(vhdl_const_bit::std_logic_bit('z'));
         sub_expr->add_expr(cmp);

         // Compare the test bit against a constant 'x'.
         type = vhdl_type::nunsigned(width);
         ref = new vhdl_var_ref(test->get_name().c_str(), type);
         ref->set_slice(new vhdl_const_int(i+base));
         cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
         cmp->add_expr(ref);
         cmp->add_expr(vhdl_const_bit::std_logic_bit('x'));
         sub_expr->add_expr(cmp);

         all->add_expr(sub_expr);
      }
   } else {
      // For a casex 'x' is a don't care, so just put 'true'.
      all->add_expr(new vhdl_const_bool(true));
   }
}

/*
 * Build the test signal to constant bits check.
 */
static void process_number(vhdl_binop_expr *all, const vhdl_var_ref *test,
                           ivl_expr_t expr, unsigned width, unsigned base,
                           bool is_casez)
{
   const char *bits = ivl_expr_bits(expr);

   bool just_dont_care = true;
   for (unsigned i = 0; i < ivl_expr_width(expr); i++) {
      switch (bits[i]) {
      case 'x':
         if (is_casez) break;
         // fallthrough
      case '?':
      case 'z':
         continue;  // Ignore these.
      }

      vhdl_binop_expr *sub_expr =
         new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
      const vhdl_type *type;
      vhdl_var_ref *ref;

      // Check if the test bit is 'z'.
      type = vhdl_type::nunsigned(width);
      ref = new vhdl_var_ref(test->get_name().c_str(), type);
      ref->set_slice(new vhdl_const_int(i+base));
      vhdl_binop_expr *cmp =
         new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
      cmp->add_expr(ref);
      cmp->add_expr(vhdl_const_bit::std_logic_bit('z'));
      sub_expr->add_expr(cmp);

      // If this is a casex statement check if the test bit is 'x'.
      if (!is_casez) {
         type = vhdl_type::nunsigned(width);
         ref = new vhdl_var_ref(test->get_name().c_str(), type);
         ref->set_slice(new vhdl_const_int(i+base));
         cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
         cmp->add_expr(ref);
         cmp->add_expr(vhdl_const_bit::std_logic_bit('x'));
         sub_expr->add_expr(cmp);
      }

      // Compare the bit against the constant value.
      type = vhdl_type::nunsigned(width);
      ref = new vhdl_var_ref(test->get_name().c_str(), type);
      ref->set_slice(new vhdl_const_int(i+base));
      cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
      cmp->add_expr(ref);
      cmp->add_expr(vhdl_const_bit::std_logic_bit(bits[i]));
      sub_expr->add_expr(cmp);

      all->add_expr(sub_expr);
      just_dont_care = false;
   }

   // If there are no bits comparisons then just put a True
   if (just_dont_care) {
      all->add_expr(new vhdl_const_bool(true));
   }
}

/*
 * Build the test signal to label signal check.
 */
static bool process_signal(vhdl_binop_expr *all, const vhdl_var_ref *test,
                           ivl_expr_t expr, unsigned width, unsigned base,
                           bool is_casez, unsigned swid, long sbase)
{
   // If the word or dimensions are not zero then we have an array.
   if (ivl_expr_oper1(expr) != 0 ||
       ivl_signal_dimensions(ivl_expr_signal(expr)) != 0) {
      error("Sorry, array selects are not currently allowed in this "
            "context.");
      return true;
   }

   unsigned ewid = ivl_expr_width(expr);
   if (sizeof(unsigned) >= sizeof(long)) {
      // Since we will be casting i (constrained by swid) to a long make sure
      // it will fit into a long. This is actually off by one, but this is the
      // best we can do since on 32 bit machines an unsigned and long are the
      // same size.
      assert(swid <= static_cast<unsigned>(numeric_limits<long>::max()));
      // We are also going to cast ewid to long so check it as well.
      assert(ewid <= static_cast<unsigned>(numeric_limits<long>::max()));
   }
   for (unsigned i = 0; i < swid; i++) {
      // Generate a comparison for this bit position
      vhdl_binop_expr *cmp;
      const vhdl_type *type;
      vhdl_var_ref *ref;

      // Check if this is an out of bounds access. If this is a casez
      // then check against a constant 'x' for the out of bound bits
      // otherwise skip the check (casex).
      if (static_cast<long>(i) + sbase >= static_cast<long>(ewid) ||
          static_cast<long>(i) + sbase < 0) {
         if (is_casez) {
            // Get the current test bit.
            type = vhdl_type::nunsigned(width);
            ref = new vhdl_var_ref(test->get_name().c_str(), type);
            ref->set_slice(new vhdl_const_int(i+base));

            // Compare the bit against 'x'.
            cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
            cmp->add_expr(ref);
            cmp->add_expr(vhdl_const_bit::std_logic_bit('x'));
            all->add_expr(cmp);
            continue;
         } else {
            // The compiler replaces a completely out of range select
            // with a constant so we know there will be at least one
            // valid bit here. We don't need a just_dont_care test.
            continue;
         }
      }

      vhdl_binop_expr *sub_expr =
         new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
      vhdl_var_ref *bit;

      // Get the current expression bit.
      // Why can we reuse the expression bit, but not the condition bit?
      type = vhdl_type::nunsigned(ivl_expr_width(expr));
      bit = new vhdl_var_ref(ivl_expr_name(expr), type);
      bit->set_slice(new vhdl_const_int(i+sbase));

      // Check if the expression bit is 'z'.
      cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
      cmp->add_expr(bit);
      cmp->add_expr(vhdl_const_bit::std_logic_bit('z'));
      sub_expr->add_expr(cmp);

      // If this is a casex statement check if the expression bit is 'x'.
      if (!is_casez) {
         cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
         cmp->add_expr(bit);
         cmp->add_expr(vhdl_const_bit::std_logic_bit('x'));
         sub_expr->add_expr(cmp);
      }

      // Check if the test bit is 'z'.
      type = vhdl_type::nunsigned(width);
      ref = new vhdl_var_ref(test->get_name().c_str(), type);
      ref->set_slice(new vhdl_const_int(i+base));
      cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
      cmp->add_expr(ref);
      cmp->add_expr(vhdl_const_bit::std_logic_bit('z'));
      sub_expr->add_expr(cmp);

      // If this is a casex statement check if the test bit is 'x'.
      if (!is_casez) {
        type = vhdl_type::nunsigned(width);
         ref = new vhdl_var_ref(test->get_name().c_str(), type);
         ref->set_slice(new vhdl_const_int(i+base));
         cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
         cmp->add_expr(ref);
         cmp->add_expr(vhdl_const_bit::std_logic_bit('x'));
         sub_expr->add_expr(cmp);
      }

      // Next check if the test and expression bits are equal.
      type = vhdl_type::nunsigned(width);
      ref = new vhdl_var_ref(test->get_name().c_str(), type);
      ref->set_slice(new vhdl_const_int(i+base));
      cmp = new vhdl_binop_expr(VHDL_BINOP_EQ, vhdl_type::boolean());
      cmp->add_expr(ref);
      cmp->add_expr(bit);
      sub_expr->add_expr(cmp);

      all->add_expr(sub_expr);
   }

   return false;
}

/*
 * These are the constructs that we allow in a casex/z label
 * expression. Returns true on failure.
 */
static bool process_expr_bits(vhdl_binop_expr *all, vhdl_var_ref *test,
                             ivl_expr_t expr, unsigned width, unsigned base,
                             bool is_casez)
{
   assert(ivl_expr_width(expr)+base <= width);

   switch (ivl_expr_type(expr)) {
   case IVL_EX_CONCAT:
      // Loop repeat number of times processing each sub element.
      for (unsigned repeat = 0; repeat < ivl_expr_repeat(expr); repeat++) {
         unsigned nparms = ivl_expr_parms(expr) - 1;
         for (unsigned parm = 0; parm <= nparms; parm++) {
            ivl_expr_t pexpr = ivl_expr_parm(expr, nparms-parm);
            if (process_expr_bits(all, test, pexpr, width, base, is_casez))
               return true;
            base += ivl_expr_width(pexpr);
         }
      }
      break;

   case IVL_EX_NUMBER:
      process_number(all, test, expr, width, base, is_casez);
      break;

   case IVL_EX_SIGNAL:
      if (process_signal(all, test, expr, width, base, is_casez,
                         ivl_expr_width(expr), 0)) return true;
      break;

   case IVL_EX_SELECT: {
      ivl_expr_t bexpr = ivl_expr_oper2(expr);
      if (ivl_expr_type(bexpr) != IVL_EX_NUMBER &&
          ivl_expr_type(bexpr) != IVL_EX_ULONG) {
         error("Sorry, only constant bit/part selects are currently allowed "
               "in this context.");
         return true;
      }
      // If the number is out of bounds or an 'x' then check against 'x'.
      if (!number_is_long(bexpr)) {
         check_against_x(all, test, expr, width, base, is_casez);
      } else if (process_signal(all, test, ivl_expr_oper1(expr), width, base,
                                is_casez, ivl_expr_width(expr),
                                get_number_as_long(bexpr))) return true;
      break;
      }

   default:
      error("Sorry, expression type %d is not currently supported.",
            ivl_expr_type(expr));
      return true;
      break;
   }

   return false;
}


/*
 * A casex/z statement cannot be directly translated to a VHDL case
 * statement as VHDL does not treat the don't-care bit as special.
 * The solution here is to generate an if statement from the casex/z
 * which compares only the non-don't-care bit positions.
 */
int draw_casezx(vhdl_procedural *proc, stmt_container *container,
                ivl_statement_t stmt, bool is_last)
{
   vhdl_var_ref *test = draw_case_test(proc, container, stmt);
   if (NULL == test)
      return 1;

   vhdl_if_stmt *result = NULL;

   int nbranches = ivl_stmt_case_count(stmt);
   bool is_casez = ivl_statement_type(stmt) == IVL_ST_CASEZ;
   for (int i = 0; i < nbranches; i++) {
      stmt_container *where = NULL;

      ivl_expr_t net = ivl_stmt_case_expr(stmt, i);
      if (net) {
         vhdl_binop_expr *all =
            new vhdl_binop_expr(VHDL_BINOP_AND, vhdl_type::boolean());
         // The net must be something we can generate a comparison for.
         if (process_expr_bits(all, test, net, ivl_expr_width(net), 0,
                               is_casez)) {
            error("%s:%d: Sorry, only case%s statements with simple "
                  "expression labels can be translated to VHDL",
                  ivl_stmt_file(stmt), ivl_stmt_lineno(stmt),
                  (is_casez ? "z" : "x"));
            delete all;
            return 1;
         }

         if (result)
            where = result->add_elsif(all);
         else {
            result = new vhdl_if_stmt(all);
            where = result->get_then_container();
         }
      }
      else {
         // This the default case and therefore the `else' branch
         assert(result);
         where = result->get_else_container();
      }

      // `where' now points to a branch of an if statement which
      // corresponds to this casex/z branch
      assert(where);
      draw_stmt(proc, where, ivl_stmt_case_stmt(stmt, i), is_last);
   }

   // Add a comment to say that this corresponds to a casex/z statement
   // as this may not be obvious
   ostringstream ss;
   ss << "Generated from case"
      << (is_casez ? 'z' : 'x')
      << " statement at " << ivl_stmt_file(stmt) << ":" << ivl_stmt_lineno(stmt);
   result->set_comment(ss.str());

   container->add_stmt(result);

   // We don't actually use the generated `test' expression
   delete test;

   return 0;
}

int draw_while(vhdl_procedural *proc, stmt_container *container,
               ivl_statement_t stmt, ivl_statement_t step=0)
{
   // Generate the body inside a temporary container before
   // generating the test
   // The reason for this is that some of the signals in the
   // test might be renamed while expanding the body (e.g. if
   // we need to generate an assignment to a constant signal)
   stmt_container tmp_container;
   int rc = draw_stmt(proc, &tmp_container, ivl_stmt_sub_stmt(stmt));
   if (rc != 0)
      return 1;
   // When we are emitting a for as a while we need to add the step
   if (step) {
      rc = draw_assign(proc, &tmp_container, step);
      if (rc != 0)
         return rc;
   }

   vhdl_expr *test = translate_expr(ivl_stmt_cond_expr(stmt));
   if (NULL == test)
      return 1;

   // The test must be a Boolean (and std_logic and (un)signed types
   // must be explicitly cast unlike in Verilog)
   vhdl_type boolean(VHDL_TYPE_BOOLEAN);
   test = test->cast(&boolean);

   emit_wait_for_0(proc, container, stmt, test);

   vhdl_while_stmt *loop = new vhdl_while_stmt(test);
   draw_stmt(proc, loop->get_container(), ivl_stmt_sub_stmt(stmt));

   // When we are emitting a for as a while we need to add the step
   if (step) {
      rc = draw_assign(proc, loop->get_container(), step);
      if (rc != 0)
         return rc;
   }

   emit_wait_for_0(proc, loop->get_container(), stmt, test);

   container->add_stmt(loop);
   return 0;
}

int draw_for_loop(vhdl_procedural *proc, stmt_container *container,
                  ivl_statement_t stmt)
{
   int rc = draw_assign(proc, container, ivl_stmt_init_stmt(stmt));
   if (rc != 0)
      return rc;

   return draw_while(proc, container, stmt, ivl_stmt_step_stmt(stmt));
}

int draw_forever(vhdl_procedural *proc, stmt_container *container,
                 ivl_statement_t stmt)
{
   vhdl_loop_stmt *loop = new vhdl_loop_stmt;
   container->add_stmt(loop);

   draw_stmt(proc, loop->get_container(), ivl_stmt_sub_stmt(stmt));

   return 0;
}

int draw_repeat(vhdl_procedural *proc, stmt_container *container,
                ivl_statement_t stmt)
{
   vhdl_expr *times = translate_expr(ivl_stmt_cond_expr(stmt));
   if (NULL == times)
      return 1;

   vhdl_type integer(VHDL_TYPE_INTEGER);
   times = times->cast(&integer);

   const char *it_name = "Verilog_Repeat";
   vhdl_for_stmt *loop =
      new vhdl_for_stmt(it_name, new vhdl_const_int(1), times);
   container->add_stmt(loop);

   draw_stmt(proc, loop->get_container(), ivl_stmt_sub_stmt(stmt));

   return 0;
}

/*
 * Tasks are difficult to translate to VHDL since they allow things
 * not allowed by VHDL's corresponding procedures (e.g. updating
 * global variables. The solution here is to expand tasks in-line.
 */
int draw_utask(vhdl_procedural *proc, stmt_container *container,
               ivl_statement_t stmt)
{
   ivl_scope_t tscope = ivl_stmt_call(stmt);

   // TODO: adding some comments to the output would be helpful

   // TODO: this completely ignores parameters!
   draw_stmt(proc, container, ivl_scope_def(tscope), false);

   return 0;
}

/*
 * Walk a nexus and find the first signal connected to it.
 * Returns the base name of that signal, or "?" if none found.
 */
string nexus_to_signal_basename(ivl_nexus_t nex)
{
   if (!nex) return "?";
   unsigned nptrs = ivl_nexus_ptrs(nex);
   for (unsigned i = 0; i < nptrs; i++) {
      ivl_nexus_ptr_t ptr = ivl_nexus_ptr(nex, i);
      ivl_signal_t sig = ivl_nexus_ptr_sig(ptr);
      if (sig)
         return ivl_signal_basename(sig);
   }
   return "?";
}

/*
 * Map a nature to its Verilog-AMS access function name.
 * "Voltage" -> "V", "Current" -> "I", etc.
 */
static string nature_to_access(ivl_nature_t nat)
{
   if (!nat) return "?";
   const char *name = ivl_nature_name(nat);
   if (!name) return "?";
   if (strcasecmp(name, "Voltage") == 0 || strcasecmp(name, "potential") == 0)
      return "V";
   if (strcasecmp(name, "Current") == 0 || strcasecmp(name, "flow") == 0)
      return "I";
   // For unknown natures, use the name directly
   return name;
}

/*
 * Recursively unparse an iverilog expression back to Verilog-A text.
 */
string analog_expr_to_str(ivl_expr_t expr)
{
   if (!expr) return "?";

   ostringstream ss;

   switch (ivl_expr_type(expr)) {
   case IVL_EX_BACCESS:
      {
         ivl_branch_t br = ivl_expr_branch(expr);
         ivl_nature_t nat = ivl_expr_nature(expr);
         string acc = nature_to_access(nat);
         string ta = nexus_to_signal_basename(ivl_branch_terminal(br, 0));
         string tb = nexus_to_signal_basename(ivl_branch_terminal(br, 1));
         ss << acc << "(" << ta;
         if (tb != ta)
            ss << ", " << tb;
         ss << ")";
      }
      break;

   case IVL_EX_BINARY:
      {
         string lhs = analog_expr_to_str(ivl_expr_oper1(expr));
         string rhs = analog_expr_to_str(ivl_expr_oper2(expr));
         char op = ivl_expr_opcode(expr);
         const char *op_str;
         switch (op) {
         case '+': op_str = " + "; break;
         case '-': op_str = " - "; break;
         case '*': op_str = " * "; break;
         case '/': op_str = " / "; break;
         case 'p': op_str = " ** "; break;
         case 'e': op_str = " == "; break;
         case 'n': op_str = " != "; break;
         case '<': op_str = " < "; break;
         case '>': op_str = " > "; break;
         case 'L': op_str = " <= "; break;
         case 'G': op_str = " >= "; break;
         case '&': op_str = " & "; break;
         case '|': op_str = " | "; break;
         case '^': op_str = " ^ "; break;
         default:
            {
               static char buf[8];
               snprintf(buf, sizeof(buf), " %c ", op);
               op_str = buf;
            }
            break;
         }
         ss << "(" << lhs << op_str << rhs << ")";
      }
      break;

   case IVL_EX_UNARY:
      {
         string operand = analog_expr_to_str(ivl_expr_oper1(expr));
         char op = ivl_expr_opcode(expr);
         switch (op) {
         case '-': ss << "-" << operand; break;
         case '!': ss << "!" << operand; break;
         case '~': ss << "~" << operand; break;
         default:  ss << (char)op << operand; break;
         }
      }
      break;

   case IVL_EX_SIGNAL:
      ss << ivl_signal_basename(ivl_expr_signal(expr));
      break;

   case IVL_EX_NUMBER:
      ss << ivl_expr_uvalue(expr);
      break;

   case IVL_EX_REALNUM:
      {
         double val = ivl_expr_dvalue(expr);
         // Use enough precision to avoid loss
         ss << std::setprecision(15) << val;
      }
      break;

   case IVL_EX_SFUNC:
      {
         const char *fname = ivl_expr_name(expr);
         // Strip '$' prefix for Verilog-A analog operators
         // ($ddt -> ddt, $idt -> idt, etc.)
         if (fname[0] == '$') fname++;
         ss << fname << "(";
         unsigned nargs = ivl_expr_parms(expr);
         for (unsigned i = 0; i < nargs; i++) {
            if (i > 0) ss << ", ";
            ss << analog_expr_to_str(ivl_expr_parm(expr, i));
         }
         ss << ")";
      }
      break;

   case IVL_EX_TERNARY:
      {
         string cond = analog_expr_to_str(ivl_expr_oper1(expr));
         string tv = analog_expr_to_str(ivl_expr_oper2(expr));
         string fv = analog_expr_to_str(ivl_expr_oper3(expr));
         ss << "(" << cond << " ? " << tv << " : " << fv << ")";
      }
      break;

   default:
      ss << "/* unsupported expr type " << ivl_expr_type(expr) << " */";
      break;
   }

   return ss.str();
}

/*
 * Recursively unparse an iverilog statement back to Verilog-A text.
 */
string analog_stmt_to_str(ivl_statement_t stmt)
{
   if (!stmt) return "";

   ostringstream ss;

   switch (ivl_statement_type(stmt)) {
   case IVL_ST_CONTRIB:
      {
         string lval = analog_expr_to_str(ivl_stmt_lexp(stmt));
         string rval = analog_expr_to_str(ivl_stmt_rval(stmt));
         ss << lval << " <+ " << rval << ";";
      }
      break;

   case IVL_ST_BLOCK:
      {
         unsigned count = ivl_stmt_block_count(stmt);
         for (unsigned i = 0; i < count; i++) {
            if (i > 0) ss << " ";
            ss << analog_stmt_to_str(ivl_stmt_block_stmt(stmt, i));
         }
      }
      break;

   case IVL_ST_CONDIT:
      {
         string cond = analog_expr_to_str(ivl_stmt_cond_expr(stmt));
         ss << "if (" << cond << ") begin ";
         ivl_statement_t t = ivl_stmt_cond_true(stmt);
         if (t) ss << analog_stmt_to_str(t);
         ss << " end";
         ivl_statement_t f = ivl_stmt_cond_false(stmt);
         if (f) {
            ss << " else begin " << analog_stmt_to_str(f) << " end";
         }
      }
      break;

   case IVL_ST_NOOP:
      break;

   default:
      ss << "/* unsupported stmt type " << ivl_statement_type(stmt) << " */";
      break;
   }

   return ss.str();
}

/*
 * Generate VHDL statements for the given Verilog statement and
 * add them to the given VHDL process. The container is the
 * location to add statements: e.g. the process body, a branch
 * of an if statement, etc.
 *
 * The flag is_last should be set if this is the final statement
 * in a block or process. It avoids generating useless `wait for 0ns'
 * statements if the next statement would be a wait anyway.
 */
int draw_stmt(vhdl_procedural *proc, stmt_container *container,
              ivl_statement_t stmt, bool is_last)
{
   assert(stmt);

   switch (ivl_statement_type(stmt)) {
   case IVL_ST_STASK:
      return draw_stask(proc, container, stmt);
   case IVL_ST_BLOCK:
      return draw_block(proc, container, stmt, is_last);
   case IVL_ST_NOOP:
      return draw_noop(proc, container, stmt);
   case IVL_ST_ASSIGN:
      return draw_assign(proc, container, stmt);
   case IVL_ST_ASSIGN_NB:
      return draw_nbassign(proc, container, stmt);
   case IVL_ST_DELAY:
   case IVL_ST_DELAYX:
      return draw_delay(proc, container, stmt);
   case IVL_ST_WAIT:
      return draw_wait(proc, container, stmt);
   case IVL_ST_CONDIT:
      return draw_if(proc, container, stmt, is_last);
   case IVL_ST_CASE:
      return draw_case(proc, container, stmt, is_last);
   case IVL_ST_WHILE:
      return draw_while(proc, container, stmt);
   case IVL_ST_FORLOOP:
      return draw_for_loop(proc, container, stmt);
   case IVL_ST_FOREVER:
      return draw_forever(proc, container, stmt);
   case IVL_ST_REPEAT:
      return draw_repeat(proc, container, stmt);
   case IVL_ST_UTASK:
      return draw_utask(proc, container, stmt);
   case IVL_ST_FORCE:
   case IVL_ST_RELEASE:
      error("force/release statements cannot be translated to VHDL");
      return 1;
   case IVL_ST_DISABLE:
      {
         // Verilog 'disable' exits a named scope block.
         // In VHDL: inside a loop this would be 'exit', but iverilog also
         // generates disable for function returns and block exits which
         // are not inside loops. Since the for-loop step is now properly
         // translated via compressed assignment expansion, the while
         // condition handles termination — so null is safe here.
         container->add_stmt(new vhdl_null_stmt());
         return 0;
      }
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
      return draw_casezx(proc, container, stmt, is_last);
   case IVL_ST_FORK:
      error("fork statement cannot be translated to VHDL");
      return 1;
   case IVL_ST_CASSIGN:
   case IVL_ST_DEASSIGN:
      error("continuous procedural assignment cannot be translated to VHDL");
      return 1;
   default:
      error("No VHDL translation for statement at %s:%d (type = %d)",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt),
            ivl_statement_type(stmt));
      return 1;
   }
}
