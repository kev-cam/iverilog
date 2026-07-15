/*
 *  VHDL code generation for expressions.
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
#include "support.hh"
#include "state.hh"

#include <iostream>
#include <sstream>
#include <cassert>
#include <cstring>

using namespace std;

/*
 * Change the signedness of a vector.
 */
static vhdl_expr *change_signedness(vhdl_expr *e, bool issigned)
{
   int msb = e->get_type()->get_msb();
   int lsb = e->get_type()->get_lsb();
   vhdl_type u(issigned ? VHDL_TYPE_SIGNED : VHDL_TYPE_UNSIGNED, msb, lsb);

   return e->cast(&u);
}

/*
 * Generate code to ensure that the VHDL expression vhd_e has the
 * same signedness as the Verilog expression vl_e.
 */
static vhdl_expr *correct_signedness(vhdl_expr *vhd_e, ivl_expr_t vl_e)
{
   bool should_be_signed = ivl_expr_signed(vl_e) != 0;

   if (vhd_e->get_type()->get_name() == VHDL_TYPE_UNSIGNED
       && should_be_signed) {
      //operand->print();
      //std::cout << "^ should be signed but is not" << std::endl;

      return change_signedness(vhd_e, true);
   }
   else if (vhd_e->get_type()->get_name() == VHDL_TYPE_SIGNED
            && !should_be_signed) {
      //operand->print();
      //std::cout << "^ should be unsigned but is not" << std::endl;

      return change_signedness(vhd_e, false);
   }
   else
      return vhd_e;
}

/*
 * Convert a constant Verilog string to a constant VHDL string.
 */
static vhdl_expr *translate_string(ivl_expr_t e)
{
   // TODO: May need to inspect or escape parts of this
   const char *str = ivl_expr_string(e);
   return new vhdl_const_string(str);
}

/*
 * A reference to a signal in an expression. It's assumed that the
 * signal has already been defined elsewhere.
 */
static vhdl_var_ref *translate_signal(ivl_expr_t e)
{
   ivl_signal_t sig = ivl_expr_signal(e);

   const vhdl_scope *scope = find_scope_for_signal(sig);
   assert(scope);

   const char *renamed = get_renamed_signal(sig).c_str();

   vhdl_decl *decl = scope->get_decl(renamed);
   assert(decl);

   // Make sure we can read from this declaration
   // E.g. if this is an `out' port then we need to make it a buffer
   decl->ensure_readable();

   // Can't generate a constant initialiser for this signal
   // later as it has already been read
   if (scope->initializing())
      decl->set_initial(NULL);

   vhdl_var_ref *ref =
      new vhdl_var_ref(renamed, new vhdl_type(*decl->get_type()));

   ivl_expr_t off;
   if (ivl_signal_array_count(sig) > 0 && (off = ivl_expr_oper1(e))) {
      // Select from an array
      vhdl_expr *vhd_off = translate_expr(off);
      if (NULL == vhd_off) {
         delete ref;
         return NULL;
      }

      vhdl_type integer(VHDL_TYPE_INTEGER);
      ref->set_slice(vhd_off->cast(&integer));
   }

   return ref;
}

/*
 * A numeric literal ends up as std_logic bit string.
 */
static vhdl_expr *translate_number(ivl_expr_t e)
{
   if (ivl_expr_width(e) == 1)
      return new vhdl_const_bit(ivl_expr_bits(e)[0]);
   else
      return new vhdl_const_bits(ivl_expr_bits(e), ivl_expr_width(e),
                                 ivl_expr_signed(e) != 0);
}

static vhdl_expr *translate_ulong(ivl_expr_t e)
{
   return new vhdl_const_int(ivl_expr_uvalue(e));
}

static vhdl_expr *translate_delay(ivl_expr_t e)
{
   return scale_time(get_active_entity(), ivl_expr_delay_val(e));
}

static vhdl_expr *translate_reduction(support_function_t f, bool neg,
                                      vhdl_expr *operand)
{
   vhdl_expr *result;
   vhdl_type_name_t scalar_type = operand->get_type()->get_name();

   if (scalar_type == VHDL_TYPE_STD_LOGIC || scalar_type == VHDL_TYPE_LOGIC3D)
      result = operand;
   else {
      require_support_function(f);

      vhdl_type *ret_type = get_sv2vhdl_mode()
         ? vhdl_type::logic3d() : vhdl_type::std_logic();
      vhdl_fcall *fcall =
         new vhdl_fcall(support_function::function_name(f), ret_type);

      if (get_sv2vhdl_mode()) {
         // logic3d_vector: pass directly, no cast needed
         fcall->add_expr(operand);
      } else {
         vhdl_type std_logic_vector(VHDL_TYPE_STD_LOGIC_VECTOR);
         fcall->add_expr(operand->cast(&std_logic_vector));
      }

      result = fcall;
   }

   if (neg) {
      vhdl_type *neg_type = get_sv2vhdl_mode()
         ? vhdl_type::logic3d() : vhdl_type::std_logic();
      return new vhdl_unaryop_expr(VHDL_UNARYOP_NOT, result, neg_type);
   }
   else
      return result;
}

static vhdl_expr *translate_unary(ivl_expr_t e)
{
   vhdl_expr *operand = translate_expr(ivl_expr_oper1(e));
   if (NULL == operand)
      return NULL;

   operand = correct_signedness(operand, e);

   char opcode = ivl_expr_opcode(e);
   switch (opcode) {
   case '!':
   case '~':
      return new vhdl_unaryop_expr
         (VHDL_UNARYOP_NOT, operand, new vhdl_type(*operand->get_type()));
   case '-':
      operand = change_signedness(operand, true);
      return new vhdl_unaryop_expr
         (VHDL_UNARYOP_NEG, operand, new vhdl_type(*operand->get_type()));
   case 'N':   // NOR
      return translate_reduction(SF_REDUCE_OR, true, operand);
   case '|':
      return translate_reduction(SF_REDUCE_OR, false, operand);
   case 'A':   // NAND
      return translate_reduction(SF_REDUCE_AND, true, operand);
   case '&':
      return translate_reduction(SF_REDUCE_AND, false, operand);
   case '^':   // XOR
      return translate_reduction(SF_REDUCE_XOR, false, operand);
   case 'X':   // XNOR
      return translate_reduction(SF_REDUCE_XNOR, false, operand);
   case '2':   // Cast to bool (4-state -> 2-state)
   case 'v':   // Cast to vec4
      // In VHDL these are no-ops — just pass through the operand
      return operand;
   default:
      error("No translation for unary opcode '%c'\n",
            ivl_expr_opcode(e));
      delete operand;
      return NULL;
   }
}

/*
 * Translate a numeric binary operator (+, -, etc.) to
 * a VHDL equivalent using the numeric_std package.
 */
static vhdl_expr *translate_numeric(vhdl_expr *lhs, vhdl_expr *rhs,
                                    vhdl_binop_t op)
{
   // May need to make either side Boolean for operators
   // to work
   vhdl_type boolean(VHDL_TYPE_BOOLEAN);
   if (lhs->get_type()->get_name() == VHDL_TYPE_BOOLEAN)
      rhs = rhs->cast(&boolean);
   else if (rhs->get_type()->get_name() == VHDL_TYPE_BOOLEAN)
      lhs = lhs->cast(&boolean);

   const vhdl_type *rtype;
   if (op == VHDL_BINOP_MULT)
      rtype = new vhdl_type(lhs->get_type()->get_name(),
                            (lhs->get_type()->get_width()*2) - 1, 0);
   else
      rtype = new vhdl_type(*lhs->get_type());
   return new vhdl_binop_expr(lhs, op, rhs, rtype);
}

static vhdl_expr *translate_relation(vhdl_expr *lhs, vhdl_expr *rhs,
                                     vhdl_binop_t op)
{
   // Generate any necessary casts
   // Arbitrarily, the RHS is casted to the type of the LHS
   vhdl_expr *r_cast = rhs->cast(lhs->get_type());

   return new vhdl_binop_expr(lhs, op, r_cast, vhdl_type::boolean());
}

/*
 * Like translate_relation but both operands must be Boolean.
 */
static vhdl_expr *translate_logical(vhdl_expr *lhs, vhdl_expr *rhs,
                                    vhdl_binop_t op)
{
   vhdl_type boolean(VHDL_TYPE_BOOLEAN);

   return translate_relation(lhs->cast(&boolean), rhs->cast(&boolean), op);
}

static vhdl_expr *translate_shift(vhdl_expr *lhs, vhdl_expr *rhs,
                                  vhdl_binop_t op)
{
   // The RHS (shift count) must be an integer. For a logic3d_vector count use
   // l3d_shcount, which saturates on overflow instead of dropping the high bits
   // like to_integer -- so a count wider than 32 bits (e.g. 2**64) correctly
   // clears/sign-fills the operand rather than reading as 0 (shift by nothing).
   vhdl_type integer(VHDL_TYPE_INTEGER);
   vhdl_expr *r_cast;
   if (rhs->get_type()
       && rhs->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      vhdl_fcall *sc = new vhdl_fcall("l3d_shcount", vhdl_type::integer());
      sc->add_expr(rhs);
      r_cast = sc;
   }
   else
      r_cast = rhs->cast(&integer);

   const vhdl_type *rtype = new vhdl_type(*lhs->get_type());

   // The sra operator is not defined on numeric_std types until
   // VHDL-2006 which is not well supported. Instead we can use
   // the shift_right function which does the same thing and
   // exists in earlier versions of numeric_std.
   if (op == VHDL_BINOP_SRA) {
      // sv2vhdl: the logic3d_vector shift_right is LOGICAL; a Verilog >>> on a
      // signed operand must sign-extend, so route to the arithmetic l3d_sra.
      const bool is_l3d = lhs->get_type()
         && lhs->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR;
      vhdl_fcall *sra = new vhdl_fcall(is_l3d ? "l3d_sra" : "shift_right", rtype);
      sra->add_expr(lhs);
      sra->add_expr(r_cast);

      return sra;
   }
   else
      return new vhdl_binop_expr(lhs, op, r_cast, rtype);
}

/*
 * The exponentiation operator in VHDL is not defined for numeric_std
 * types. We can get around this by converting the operands to integers,
 * performing the operation, then converting the result back to the
 * original type. This will work OK in simulation but certainly will not
 * synthesise unless the operands are constant.
 *
 * However, even this does not work quite correctly. The Integer type in
 * VHDL is signed and usually only 32 bits, therefore any result larger
 * than this will overflow and raise an exception. I can't see a way
 * around this at the moment.
 */
static vhdl_expr *translate_power(ivl_expr_t e, vhdl_expr *lhs, vhdl_expr *rhs)
{
   vhdl_type integer(VHDL_TYPE_INTEGER);
   vhdl_expr *lhs_int = lhs->cast(&integer);
   vhdl_expr *rhs_int = rhs->cast(&integer);

   vhdl_expr *result = new vhdl_binop_expr(lhs_int, VHDL_BINOP_POWER, rhs_int,
                                           vhdl_type::integer());

   int width = ivl_expr_width(e);
   const char *func = ivl_expr_signed(e) ? "To_Signed" : "To_Unsigned";
   const vhdl_type *type = ivl_expr_signed(e)
      ? vhdl_type::nsigned(width) : vhdl_type::nunsigned(width);

   vhdl_fcall *conv = new vhdl_fcall(func, type);
   conv->add_expr(result);
   conv->add_expr(new vhdl_const_int(width));

   return conv;
}

static vhdl_expr *translate_binary(ivl_expr_t e)
{
   vhdl_expr *lhs = translate_expr(ivl_expr_oper1(e));
   if (NULL == lhs)
      return NULL;

   vhdl_expr *rhs = translate_expr(ivl_expr_oper2(e));
   if (NULL == rhs) {
      delete lhs;
      return NULL;
   }

   int lwidth = lhs->get_type()->get_width();
   int rwidth = rhs->get_type()->get_width();
   int result_width = ivl_expr_width(e);

   // There's a funny corner-case where both the LHS and RHS are constant
   // single bit numbers and the VHDL compiler can't decide between the
   // std_ulogic and bit overloads of various operators
   const bool lnumber = ivl_expr_type(ivl_expr_oper1(e)) == IVL_EX_NUMBER;
   const bool rnumber = ivl_expr_type(ivl_expr_oper2(e)) == IVL_EX_NUMBER;
   if (lwidth == 1 && rwidth == 1 && lnumber && rnumber) {
      // It's sufficient to qualify only one side
      vhdl_fcall *lqual = new vhdl_fcall("std_logic'", lhs->get_type());
      lqual->add_expr(lhs);

      lhs = lqual;
   }

   // For === and !== we need to compare std_logic_vectors
   // rather than signeds
   vhdl_type std_logic_vector(VHDL_TYPE_STD_LOGIC_VECTOR, result_width-1, 0);
   vhdl_type_name_t ltype = lhs->get_type()->get_name();
   vhdl_type_name_t rtype = rhs->get_type()->get_name();
   bool vectorop =
      (ltype == VHDL_TYPE_SIGNED || ltype == VHDL_TYPE_UNSIGNED) &&
      (rtype == VHDL_TYPE_SIGNED || rtype == VHDL_TYPE_UNSIGNED);

   // May need to resize the left or right hand side or change the
   // signedness
   if (vectorop) {
      if (lwidth < rwidth)
         lhs = lhs->resize(rwidth);
      else if (rwidth < lwidth)
         rhs = rhs->resize(lwidth);

      lhs = correct_signedness(lhs, ivl_expr_oper1(e));
      rhs = correct_signedness(rhs, ivl_expr_oper2(e));
   }

   // sv2vhdl: logic3d_vector relational/div/mod operators are unsigned-only.
   // A Verilog signed context (BOTH operands signed) needs the sign bit
   // honoured, so route to the dedicated signed helpers. numeric_std's signed
   // compare/div sign-extends the shorter operand, so no pre-resize is needed.
   if (get_sv2vhdl_mode()
       && ivl_expr_signed(ivl_expr_oper1(e))
       && ivl_expr_signed(ivl_expr_oper2(e))
       && lhs->get_type()
       && lhs->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      const char *cf = 0, *vf = 0;
      switch (ivl_expr_opcode(e)) {
      case '<': cf = "l3d_lt_s"; break;
      case '>': cf = "l3d_gt_s"; break;
      case 'L': cf = "l3d_le_s"; break;
      case 'G': cf = "l3d_ge_s"; break;
      case '/': vf = "l3d_div_s"; break;
      case '%': vf = "l3d_mod_s"; break;
      }
      if (cf) {
         vhdl_fcall *f = new vhdl_fcall(cf, vhdl_type::boolean());
         f->add_expr(lhs); f->add_expr(rhs);
         return f;
      }
      if (vf) {
         vhdl_fcall *f = new vhdl_fcall(vf, new vhdl_type(*lhs->get_type()));
         f->add_expr(lhs); f->add_expr(rhs);
         return f;
      }
   }

   vhdl_expr *result;
   switch (ivl_expr_opcode(e)) {
   case '+':
      result = translate_numeric(lhs, rhs, VHDL_BINOP_ADD);
      break;
   case '-':
      result = translate_numeric(lhs, rhs, VHDL_BINOP_SUB);
      break;
   case '*':
      result = translate_numeric(lhs, rhs, VHDL_BINOP_MULT);
      break;
   case '/':
      result = translate_numeric(lhs, rhs, VHDL_BINOP_DIV);
      break;
   case '%':
      result = translate_numeric(lhs, rhs, VHDL_BINOP_MOD);
      break;
   case 'e':
      result = translate_relation(lhs, rhs, VHDL_BINOP_EQ);
      break;
   case 'E':
      if (vectorop)
         result = translate_relation(lhs->cast(&std_logic_vector),
                                     rhs->cast(&std_logic_vector), VHDL_BINOP_EQ);
      else
         result = translate_relation(lhs, rhs, VHDL_BINOP_EQ);
      break;
   case 'n':
      result = translate_relation(lhs, rhs, VHDL_BINOP_NEQ);
      break;
   case 'N':
      if (vectorop)
         result = translate_relation(lhs->cast(&std_logic_vector),
                                     rhs->cast(&std_logic_vector), VHDL_BINOP_NEQ);
      else
         result = translate_relation(lhs, rhs, VHDL_BINOP_NEQ);
      break;
   case '&':    // Bitwise AND
      result = translate_numeric(lhs, rhs, VHDL_BINOP_AND);
      break;
   case 'a':    // Logical AND
      result = translate_logical(lhs, rhs, VHDL_BINOP_AND);
      break;
   case 'A':    // Bitwise NAND
      result = translate_numeric(lhs, rhs, VHDL_BINOP_NAND);
      break;
   case 'O':    // Bitwise NOR
      result = translate_numeric(lhs, rhs, VHDL_BINOP_NOR);
      break;
   case 'X':    // Bitwise XNOR
      result = translate_numeric(lhs, rhs, VHDL_BINOP_XNOR);
      break;
   case '|':    // Bitwise OR
      result = translate_numeric(lhs, rhs, VHDL_BINOP_OR);
      break;
   case 'o':    // Logical OR
      result = translate_logical(lhs, rhs, VHDL_BINOP_OR);
      break;
   case '<':
      result = translate_relation(lhs, rhs, VHDL_BINOP_LT);
      break;
   case 'L':
      result = translate_relation(lhs, rhs, VHDL_BINOP_LEQ);
      break;
   case '>':
      result = translate_relation(lhs, rhs, VHDL_BINOP_GT);
      break;
   case 'G':
      result = translate_relation(lhs, rhs, VHDL_BINOP_GEQ);
      break;
   case 'l':
      result = translate_shift(lhs, rhs, VHDL_BINOP_SL);
      break;
   case 'r':
      result = translate_shift(lhs, rhs, VHDL_BINOP_SR);
      break;
   case 'R':    // Arithmetic right shift
      // Verilog only actually performs a signed shift if the
      // argument being shifted is signed, otherwise it defaults
      // to a normal shift
      if (ivl_expr_signed(ivl_expr_oper1(e)))
         result = translate_shift(lhs, rhs, VHDL_BINOP_SRA);
      else
         result = translate_shift(lhs, rhs, VHDL_BINOP_SR);
      break;
   case '^':
      result = translate_numeric(lhs, rhs, VHDL_BINOP_XOR);
      break;
   case 'p':    // Power
      result = translate_power(e, lhs, rhs);
      break;
   default:
      error("No translation for binary opcode '%c'\n",
            ivl_expr_opcode(e));
      delete lhs;
      delete rhs;
      return NULL;
   }

   if (NULL == result)
      return NULL;

   if (vectorop) {
      result = correct_signedness(result, e);

      int actual_width = result->get_type()->get_width();
      if (actual_width != result_width) {
         //result->print();
         //std::cout << "^ should be " << result_width << " but is " << actual_width << std::endl;
      }
   }

   return result;
}

static vhdl_expr *translate_select(ivl_expr_t e)
{
   vhdl_expr *from = translate_expr(ivl_expr_oper1(e));
   if (NULL == from)
      return NULL;

   ivl_expr_t o2 = ivl_expr_oper2(e);
   if (o2) {
      vhdl_expr *base = translate_expr(ivl_expr_oper2(e));
      if (NULL == base)
         return NULL;

      vhdl_var_ref *from_var_ref = dynamic_cast<vhdl_var_ref*>(from);
      if (NULL == from_var_ref) {
         // We can't directly select bits from something that's not
         // a variable reference in VHDL, but we can emulate the
         // effect with a shift and a resize

         vhdl_expr *shifted;
         if (ivl_expr_signed(ivl_expr_oper1(e))) {
            vhdl_fcall *sra = new vhdl_fcall("shift_right", from->get_type());
            sra->add_expr(from);
            sra->add_expr(base->to_integer());
            shifted = sra;
         }
         else
            shifted = new vhdl_binop_expr(from, VHDL_BINOP_SR, base->to_integer(),
                                          from->get_type());

         // Truncate the shifted value to the select width (LSB-aligned) so a
         // 1-bit select yields a single bit, not the full source vector.
         return shifted->resize(ivl_expr_width(e));
      }
      else if (ivl_expr_type(ivl_expr_oper1(e)) == IVL_EX_SIGNAL
               && ivl_signal_data_type(ivl_expr_signal(ivl_expr_oper1(e)))
                     == IVL_VT_QUEUE) {
         // SystemVerilog queue element select q[i]: emit an array-element
         // access into the ring buffer, q((head + i) mod DEPTH) — NOT a bit
         // slice. Return a fresh ref typed as the element (logic3d_vector) so
         // operators on q[i] (e.g. q[0] + ...) resolve against the element.
         delete from_var_ref;
         string q(get_renamed_signal(ivl_expr_signal(ivl_expr_oper1(e))));
         vhdl_type integer(VHDL_TYPE_INTEGER);
         vhdl_expr *ofs = new vhdl_binop_expr(
            new vhdl_var_ref((q + "_head").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
            VHDL_BINOP_ADD, base->cast(&integer),
            new vhdl_type(VHDL_TYPE_INTEGER));
         vhdl_expr *idx = new vhdl_binop_expr(
            ofs, VHDL_BINOP_MOD, new vhdl_const_int(64),
            new vhdl_type(VHDL_TYPE_INTEGER));
         // Use the queue's true element width (not ivl_expr_width, which can be
         // 1 in some contexts) so the element type is consistently a vector.
         ivl_signal_t qsig = ivl_expr_signal(ivl_expr_oper1(e));
         ivl_type_t et = ivl_type_element(ivl_signal_net_type(qsig));
         int ew = et ? ivl_type_packed_width(et) : ivl_expr_width(e);
         if (ew < 1) ew = 1;
         // Type the ref as the queue ARRAY: set_slice on an array yields the
         // element type (logic3d_vector) via get_base(); a vector-typed ref
         // would instead degrade to a bit (logic3d) and mistype q[i].
         vhdl_type *elem = vhdl_type::logic3d_vector(ew - 1, 0);
         vhdl_type *arr = vhdl_type::array_of(elem, q + "_QType", 63, 0);
         vhdl_var_ref *qref = new vhdl_var_ref(q.c_str(), arr);
         qref->set_slice(idx);
         return qref;
      }
      else if (from_var_ref->get_type()->get_name() != VHDL_TYPE_STD_LOGIC) {
         // We can use the more idiomatic VHDL slice notation on a
         // single variable reference
         vhdl_type integer(VHDL_TYPE_INTEGER);
         from_var_ref->set_slice(base->cast(&integer), ivl_expr_width(e) - 1);
         return from_var_ref;
      }
      else {
         // Make sure we're not trying to select more than one bit
         // from a std_logic (this shouldn't actually happen)
         if (ivl_expr_width(e) > 1) {
            error("%s:%d: trying to select more than one bit from a std_logic",
                  ivl_expr_file(e), ivl_expr_lineno(e));
            return NULL;
         }
         else
            return from_var_ref;
      }
   }
   else {
      vhdl_expr *padded = correct_signedness(from, e);
      // sv2vhdl: a widening pad-select (oper2 == NULL) of a signed logic3d
      // value must sign-extend, but resize() zero-fills the logic3d_vector.
      // Route signed widenings through l3d_resize_s. (Unsigned widenings and
      // truncations keep zero-fill / low-bit semantics, which resize() gives.)
      if (get_sv2vhdl_mode() && ivl_expr_signed(e)
          && padded->get_type()
          && padded->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR
          && ivl_expr_width(e) > padded->get_type()->get_width()) {
         vhdl_fcall *f = new vhdl_fcall("l3d_resize_s",
            vhdl_type::logic3d_vector(ivl_expr_width(e) - 1, 0));
         f->add_expr(padded);
         f->add_expr(new vhdl_const_int(ivl_expr_width(e)));
         return f;
      }
      return padded->resize(ivl_expr_width(e));
   }
}

template <class T>
static T *translate_parms(T *t, ivl_expr_t e)
{
   int nparams = ivl_expr_parms(e);
   for (int i = 0; i < nparams; i++) {
      vhdl_expr *param = translate_expr(ivl_expr_parm(e, i));
      if (NULL == param)
         return NULL;

      t->add_expr(param);
   }

   return t;
}

static vhdl_expr *translate_ufunc(ivl_expr_t e)
{
   ivl_scope_t defscope = ivl_expr_def(e);
   ivl_scope_t parentscope = ivl_scope_parent(defscope);
   assert(ivl_scope_type(parentscope) == IVL_SCT_MODULE);

   // A function is always declared in a module, which should have
   // a corresponding entity by this point: so we can get type
   // information, etc. from the declaration
   vhdl_entity *parent_ent = find_entity(parentscope);
   assert(parent_ent);

   const char *funcname = ivl_scope_tname(defscope);

   const vhdl_type *rettype =
      vhdl_type::type_for(ivl_expr_width(e), ivl_expr_signed(e) != 0);
   vhdl_fcall *fcall = new vhdl_fcall(funcname, rettype);

   int nparams = ivl_expr_parms(e);
   for (int i = 0; i < nparams; i++) {
      vhdl_expr *param = translate_expr(ivl_expr_parm(e, i));
      if (NULL == param) {
         delete fcall;
         return NULL;
      }

      // Ensure the parameter has the correct VHDL type
      // Parameter number is i + 1 since 0th parameter is return value
      ivl_signal_t param_sig = ivl_scope_port(defscope, i + 1);
      vhdl_type *param_type =
         vhdl_type::type_for(ivl_signal_width(param_sig),
                             ivl_signal_signed(param_sig) != 0);

      fcall->add_expr(param->cast(param_type));
      delete param_type;
   }

   return fcall;
}

static vhdl_expr *translate_ternary(ivl_expr_t e)
{
   support_function_t sf;
   int width = ivl_expr_width(e);
   bool issigned = ivl_expr_signed(e) != 0;
   if (width == 1)
      sf = SF_TERNARY_LOGIC;
   else if (issigned)
      sf = SF_TERNARY_SIGNED;
   else
      sf = SF_TERNARY_UNSIGNED;

   require_support_function(sf);

   vhdl_expr *test = translate_expr(ivl_expr_oper1(e));
   vhdl_expr *true_part = translate_expr(ivl_expr_oper2(e));
   vhdl_expr *false_part = translate_expr(ivl_expr_oper3(e));
   if (!test || !true_part || !false_part)
      return NULL;

   vhdl_type boolean(VHDL_TYPE_BOOLEAN);
   test = test->cast(&boolean);

   vhdl_fcall *fcall =
      new vhdl_fcall(support_function::function_name(sf),
                     vhdl_type::type_for(width, issigned));
   fcall->add_expr(test);
   fcall->add_expr(true_part);
   fcall->add_expr(false_part);

   return fcall;
}

static vhdl_expr *translate_concat(ivl_expr_t e)
{
   const vhdl_type *rtype =
      vhdl_type::type_for(ivl_expr_width(e), ivl_expr_signed(e) != 0);
   vhdl_binop_expr *concat = new vhdl_binop_expr(VHDL_BINOP_CONCAT, rtype);

   int nrepeat = ivl_expr_repeat(e);
   while (nrepeat--) {
      int nparams = ivl_expr_parms(e);
      for (int i = 0; i < nparams; i++) {
         vhdl_expr *param = translate_expr(ivl_expr_parm(e, i));
         if (NULL == param)
            return NULL;

         // Concatenation elements must be std_logic compatible;
         // cast boolean comparison results to std_logic
         if (param->get_type()
             && param->get_type()->get_name() == VHDL_TYPE_BOOLEAN) {
            param = param->cast(vhdl_type::std_logic());
         }
         concat->add_expr(param);
      }
   }

   return concat;
}

// A VHDL time literal for ONE simulation tick. Delays are emitted as a count of
// ticks in this unit (see set_time_units), so `now' divided by it is the tick
// count. Note the base is deliberately compressed rather than true SI -- see
// vhdl_tick_unit().
static std::string tick_literal()
{
   const int prec = ivl_design_time_precision(get_vhdl_design());
   return std::string("1 ") + time_unit_name(vhdl_tick_unit(prec));
}

// A VHDL time literal for one unit of the ACTIVE SCOPE's Verilog timescale,
// expressed in the same tick base the delays use. A scope's unit spans
// 10^(units - precision) ticks (ivl_scope_time_units is a signed power of 10:
// -9 = 1 ns, -8 = 10 ns, ...), so `now' divided by this yields the Verilog time
// count in that scope's units.
//
// This must be derived from the tick base, NOT from the true SI size of the
// scope's unit: the emitted delays live in the compressed base, so an
// independently-computed SI literal disagrees with them whenever the two scales
// differ (which is exactly what made $time/$simtime read 0 in a module with no
// timescale).
static std::string scope_unit_literal(int units)
{
   const int prec = ivl_design_time_precision(get_vhdl_design());
   uint64_t ticks = 1;
   for (int k = 0; k < units - prec; k++)
      ticks *= 10;
   ostringstream ss;
   ss << ticks << " " << time_unit_name(vhdl_tick_unit(prec));
   return ss.str();
}

// $time / $stime: the current simulation time scaled to the calling scope's
// time units. The scope's units come from the scope-keyed store
// (set_active_scope in draw_process).
vhdl_expr *translate_sfunc_time(ivl_expr_t)
{
   string e = "(now / (" + scope_unit_literal(active_time_units()) + "))";
   return new vhdl_var_ref(e.c_str(), vhdl_type::integer());
}

vhdl_expr *translate_sfunc_stime(ivl_expr_t)
{
   string e = "(now / (" + scope_unit_literal(active_time_units()) + "))";
   return new vhdl_var_ref(e.c_str(), vhdl_type::integer());
}

vhdl_expr *translate_sfunc_simtime(ivl_expr_t)
{
   // $simtime is the raw simulation time: a count of ticks, unscaled by any
   // scope's timescale units.
   string e = "(now / (" + tick_literal() + "))";
   return new vhdl_var_ref(e.c_str(), vhdl_type::integer());
}

vhdl_expr *translate_sfunc_random(ivl_expr_t e)
{
   // sv2vhdl mode: $random(seed) -> sv_random(seed), a deterministic seeded
   // value. The seed update (seed = sv_random(seed)) is emitted by draw_assign,
   // so this stays a plain function (VHDL functions can't have inout params)
   // and composes with any lvalue and a signal- or variable-class seed.
   // NB: $fopen also routes here (it reuses the stub-0 path), so gate on the
   // name actually being $random before treating arg 0 as a seed.
   if (get_sv2vhdl_mode() && strcmp(ivl_expr_name(e), "$random") == 0
       && ivl_expr_parms(e) >= 1) {
      vhdl_expr *seed = translate_expr(ivl_expr_parm(e, 0));
      if (seed) {
         const int w = ivl_expr_width(e);
         vhdl_fcall *f = new vhdl_fcall("sv_random",
                                        vhdl_type::logic3d_vector(w - 1, 0));
         f->add_expr(seed);
         return f;
      }
   }
   cerr << "warning: no translation for $random (returning 0)" << endl;
   vhdl_expr *result = new vhdl_const_int(0);
   result->set_comment("$random not supported, returned 0 instead!");
   return result;
}

vhdl_expr *translate_sfunc_fopen(ivl_expr_t)
{
   cerr << "warning: no translation for $fopen (returning 0)" << endl;
   vhdl_expr *result = new vhdl_const_int(0);
   result->set_comment("$fopen not supported, returned 0 instead!");
   return result;
}

/*
 * `$get_val(arr, idx0, idx1, …)` returns the inner-element slice at
 * `arr[idx0][idx1]…`. Used as an escape hatch for chained-RHS reads on
 * packed arrays where iverilog's elaborator rejects non-constant outer
 * indices. Emitted as `arr(flat_offset + W-1 downto flat_offset)` where
 * the flat offset and width come from the signal's packed dims.
 */
vhdl_expr *translate_sfunc_get_val(ivl_expr_t e)
{
   const unsigned count = ivl_expr_parms(e);
   if (count < 2) {
      error("$get_val requires at least 2 args (arr, idx)");
      return NULL;
   }
   ivl_expr_t arr_e = ivl_expr_parm(e, 0);
   if (!arr_e || ivl_expr_type(arr_e) != IVL_EX_SIGNAL) {
      error("first arg to $get_val must be a signal");
      return NULL;
   }
   ivl_signal_t sig = ivl_expr_signal(arr_e);
   string signame(get_renamed_signal(sig));

   const int nidx = count - 1;
   const unsigned packed_dims = ivl_signal_packed_dimensions(sig);
   if (nidx < 1 || (unsigned)nidx > packed_dims) {
      error("$get_val index count doesn't match packed dims");
      return NULL;
   }

   std::vector<int> dim_size(packed_dims, 1);
   for (unsigned d = 0; d < packed_dims; d++) {
      int msb = ivl_signal_packed_msb(sig, d);
      int lsb = ivl_signal_packed_lsb(sig, d);
      dim_size[d] = (msb >= lsb ? msb - lsb : lsb - msb) + 1;
   }
   // Inner-element width = product of dim sizes for the unindexed (inner) dims.
   int inner_width = 1;
   for (int d = nidx; d < (int)packed_dims; d++)
      inner_width *= dim_size[d];

   vhdl_type integer(VHDL_TYPE_INTEGER);
   vhdl_expr *flat = NULL;
   for (int p = 0; p < nidx; p++) {
      ivl_expr_t idx_e = ivl_expr_parm(e, p + 1);
      vhdl_expr *idx = translate_expr(idx_e);
      if (!idx) return NULL;
      idx = idx->cast(&integer);
      // Subtract this dim's lsb so the expression is 0-based against
      // iverilog's flat storage. E.g. for `[31:1]` packed dim, source
      // indexes j=1..31 must map to storage offsets 0..30.
      int msb = ivl_signal_packed_msb(sig, p);
      int lsb = ivl_signal_packed_lsb(sig, p);
      int dim_lsb = (msb >= lsb) ? lsb : msb;
      if (dim_lsb != 0) {
         vhdl_expr *off = new vhdl_const_int(dim_lsb);
         idx = new vhdl_binop_expr(idx, VHDL_BINOP_SUB, off,
                                    new vhdl_type(VHDL_TYPE_INTEGER));
      }
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

   const vhdl_type *result_type =
      vhdl_type::type_for(inner_width, ivl_signal_signed(sig) != 0);
   vhdl_var_ref *ref =
      new vhdl_var_ref(signame, new vhdl_type(*result_type));
   ref->set_slice(flat, inner_width - 1);
   return ref;
}

vhdl_expr *translate_sfunc(ivl_expr_t e)
{
   const char *name = ivl_expr_name(e);
   if (strcmp(name, "$time") == 0)
      return translate_sfunc_time(e);
   else if (strcmp(name, "$stime") == 0)
      return translate_sfunc_stime(e);
   else if (strcmp(name, "$simtime") == 0)
      return translate_sfunc_simtime(e);
   else if (strcmp(name, "$random") == 0)
      return translate_sfunc_random(e);
   else if (strcmp(name, "$fopen") == 0)
      return translate_sfunc_random(e);
   else if (strcmp(name, "$get_val") == 0)
      return translate_sfunc_get_val(e);
   else if (strcmp(name, "$urandom") == 0 || strcmp(name, "$urandom_range") == 0) {
      // Map to SV2VHDL.SV_MATH_PKG.random (iverilog's own RNG via VHPIDIRECT).
      // It returns integer; for a multi-bit context wrap it as a logic3d_vector
      // of the context width so the assign target type matches.
      vhdl_fcall *r = new vhdl_fcall("random", new vhdl_type(VHDL_TYPE_INTEGER));
      const int w = ivl_expr_width(e);
      if (w <= 1)
         return r;   // integer is fine in 1-bit / comparison contexts
      // random returns a SIGNED int32 that may be negative, so to_unsigned()
      // would raise a NATURAL-range error. Reinterpret the bits instead:
      // unsigned(to_signed(random, w)).
      vhdl_fcall *ts = new vhdl_fcall("to_signed", vhdl_type::nsigned(w));
      ts->add_expr(r);
      ts->add_expr(new vhdl_const_int(w));
      vhdl_fcall *tu = new vhdl_fcall("unsigned", vhdl_type::nunsigned(w));
      tu->add_expr(ts);
      vhdl_fcall *l3 = new vhdl_fcall("unsigned_to_l3d",
                                      vhdl_type::logic3d_vector(w - 1, 0));
      l3->add_expr(tu);
      return l3;
   }
   else if (strcmp(name, "$size") == 0) {
      // SystemVerilog queue.size() -> ring-buffer (tail - head). Return it as a
      // logic3d_vector of the expression width so it composes with the logic3d
      // arithmetic the rest of the (sv2vhdl-mode) testbench is built from.
      ivl_expr_t qe = ivl_expr_parm(e, 0);
      if (!qe || ivl_expr_type(qe) != IVL_EX_SIGNAL) {
         error("$size argument must be a signal");
         return NULL;
      }
      string q(get_renamed_signal(ivl_expr_signal(qe)));
      vhdl_expr *diff = new vhdl_binop_expr(
         new vhdl_var_ref((q + "_tail").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
         VHDL_BINOP_SUB,
         new vhdl_var_ref((q + "_head").c_str(), new vhdl_type(VHDL_TYPE_INTEGER)),
         new vhdl_type(VHDL_TYPE_INTEGER));
      const int w = ivl_expr_width(e);
      if (w <= 1)
         return diff;
      vhdl_fcall *tu = new vhdl_fcall("to_unsigned", vhdl_type::nunsigned(w));
      tu->add_expr(diff);
      tu->add_expr(new vhdl_const_int(w));
      vhdl_fcall *l3 = new vhdl_fcall("unsigned_to_l3d",
                                      vhdl_type::logic3d_vector(w - 1, 0));
      l3->add_expr(tu);
      return l3;
   }
   else {
      error("No translation for system function %s", name);
      return NULL;
   }
}

/*
 * Generate a VHDL expression from a Verilog expression.
 */
vhdl_expr *translate_expr(ivl_expr_t e)
{
   assert(e);
   ivl_expr_type_t type = ivl_expr_type(e);

   switch (type) {
   case IVL_EX_STRING:
      return translate_string(e);
   case IVL_EX_SIGNAL:
      return translate_signal(e);
   case IVL_EX_NUMBER:
      return translate_number(e);
   case IVL_EX_ULONG:
      return translate_ulong(e);
   case IVL_EX_UNARY:
      return translate_unary(e);
   case IVL_EX_BINARY:
      return translate_binary(e);
   case IVL_EX_SELECT:
      return translate_select(e);
   case IVL_EX_UFUNC:
      return translate_ufunc(e);
   case IVL_EX_TERNARY:
      return translate_ternary(e);
   case IVL_EX_CONCAT:
      return translate_concat(e);
   case IVL_EX_SFUNC:
      return translate_sfunc(e);
   case IVL_EX_DELAY:
      return translate_delay(e);
   case IVL_EX_REALNUM:
      error("No VHDL translation for real expression at %s:%d",
            ivl_expr_file(e), ivl_expr_lineno(e));
      return NULL;
   default:
      error("No VHDL translation for expression at %s:%d (type = %d)",
            ivl_expr_file(e), ivl_expr_lineno(e), type);
      return NULL;
   }
}

/*
 * Translate an expression into a time. This is achieved simply
 * by multiplying the expression by 1ns.
 */
vhdl_expr *translate_time_expr(ivl_expr_t e)
{
   vhdl_expr *time = translate_expr(e);
   if (NULL == time)
      return NULL;

   if (time->get_type()->get_name() != VHDL_TYPE_TIME) {
      vhdl_type integer(VHDL_TYPE_INTEGER);
      time = time->cast(&integer);

      vhdl_expr *ns1 = scale_time(get_active_entity(), 1);
      return new vhdl_binop_expr(time, VHDL_BINOP_MULT, ns1,
                                 vhdl_type::time());
   }
   else // Translating IVL_EX_DELAY will always return a time type
      return time;
}
