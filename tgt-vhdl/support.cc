/*
 *  Support functions for VHDL output.
 *
 *  Copyright (C) 2008-2009  Nick Gasson (nick@nickg.me.uk)
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

#include <cassert>
#include <cstring>
#include <iostream>

void require_support_function(support_function_t f)
{
   vhdl_scope *scope = get_active_entity()->get_arch()->get_scope();
   if (!scope->have_declared(support_function::function_name(f)))
      scope->add_decl(new support_function(f));
}

const char *support_function::function_name(support_function_t type)
{
   switch (type) {
   case SF_UNSIGNED_TO_BOOLEAN: return "Unsigned_To_Boolean";
   case SF_SIGNED_TO_BOOLEAN:   return "Signed_To_Boolean";
   case SF_BOOLEAN_TO_LOGIC:    return "Boolean_To_Logic";
   case SF_REDUCE_OR:           return "Reduce_OR";
   case SF_REDUCE_AND:          return "Reduce_AND";
   case SF_REDUCE_XOR:          return "Reduce_XOR";
   case SF_REDUCE_XNOR:         return "Reduce_XNOR";
   case SF_TERNARY_LOGIC:       return "Ternary_Logic";
   case SF_TERNARY_UNSIGNED:    return "Ternary_Unsigned";
   case SF_TERNARY_SIGNED:      return "Ternary_Signed";
   case SF_LOGIC_TO_INTEGER:    return "Logic_To_Integer";
   case SF_SIGNED_TO_LOGIC:     return "Signed_To_Logic";
   case SF_UNSIGNED_TO_LOGIC:   return "Unsigned_To_Logic";
   case SF_TIME_FIELD:          return "Verilog_Time_Field";
   case SF_REM_SIGNED:          return "Verilog_Rem_S";
   case SF_REAL_G:              return "Verilog_Real_G";
   default:
      assert(false);
   }
   return "Invalid";
}

vhdl_type *support_function::function_type(support_function_t type)
{
   switch (type) {
   case SF_UNSIGNED_TO_BOOLEAN:
   case SF_SIGNED_TO_BOOLEAN:
      return vhdl_type::boolean();
   case SF_BOOLEAN_TO_LOGIC:
   case SF_REDUCE_OR:
   case SF_REDUCE_AND:
   case SF_REDUCE_XOR:
   case SF_REDUCE_XNOR:
   case SF_TERNARY_LOGIC:
   case SF_SIGNED_TO_LOGIC:
   case SF_UNSIGNED_TO_LOGIC:
      return get_sv2vhdl_mode() ? vhdl_type::logic3d() : vhdl_type::std_logic();
   case SF_TERNARY_SIGNED:
      // sv2vhdl carries signed values as logic3d_vector too (signedness lives
      // in the l3d_*_s operators), so the branches arrive as logic3d_vector.
      return get_sv2vhdl_mode()
         ? new vhdl_type(VHDL_TYPE_LOGIC3D_VECTOR)
         : new vhdl_type(VHDL_TYPE_SIGNED);
   case SF_TERNARY_UNSIGNED:
      return get_sv2vhdl_mode()
         ? new vhdl_type(VHDL_TYPE_LOGIC3D_VECTOR)
         : new vhdl_type(VHDL_TYPE_UNSIGNED);
   case SF_LOGIC_TO_INTEGER:
      return vhdl_type::integer();
   case SF_TIME_FIELD:
   case SF_REAL_G:
      return vhdl_type::string();
   case SF_REM_SIGNED:
      return new vhdl_type(VHDL_TYPE_LOGIC3D_VECTOR);
   }
   assert(false);
   return vhdl_type::boolean();
}

void support_function::emit_ternary(std::ostream &of, int level) const
{
   of << nl_string(level) << "begin" << nl_string(indent(level))
      << "if T then return X; else return Y; end if;";
}

void support_function::emit_reduction(std::ostream &of, int level,
                                      const char *op, char unit) const
{
   if (get_sv2vhdl_mode()) {
      // logic3d version: use l3d_* functions instead of VHDL operators
      const char *l3d_fn;
      const char *l3d_unit;
      if (strcmp(op, "or") == 0)   { l3d_fn = "l3d_or";  l3d_unit = "L3D_0"; }
      else if (strcmp(op, "and") == 0) { l3d_fn = "l3d_and"; l3d_unit = "L3D_1"; }
      else if (strcmp(op, "xor") == 0) { l3d_fn = "l3d_xor"; l3d_unit = "L3D_0"; }
      else { l3d_fn = "l3d_xor"; l3d_unit = "L3D_1"; } // xnor

      of << "(X : logic3d_vector) return logic3d is"
         << nl_string(indent(level))
         << "variable R : logic3d := " << l3d_unit << ";" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "for I in X'Range loop" << nl_string(indent(indent(level)))
         << "R := " << l3d_fn << "(X(I), R);" << nl_string(indent(level))
         << "end loop;" << nl_string(indent(level))
         << "return R;";
   }
   else {
      of << "(X : std_logic_vector) return std_logic is"
         << nl_string(indent(level))
         << "variable R : std_logic := '" << unit << "';" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "for I in X'Range loop" << nl_string(indent(indent(level)))
         << "R := X(I) " << op << " R;" << nl_string(indent(level))
         << "end loop;" << nl_string(indent(level))
         << "return R;";
   }
}

void support_function::emit(std::ostream &of, int level) const
{
   of << nl_string(level) << "function " << function_name(type_);

   switch (type_) {
   case SF_UNSIGNED_TO_BOOLEAN:
      of << "(X : unsigned) return Boolean is" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "return X /= To_Unsigned(0, X'Length);";
      break;
   case SF_SIGNED_TO_BOOLEAN:
      of << "(X : signed) return Boolean is" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "return X /= To_Signed(0, X'Length);";
      break;
   case SF_BOOLEAN_TO_LOGIC:
      if (get_sv2vhdl_mode()) {
         of << "(B : Boolean) return logic3d is" << nl_string(level)
            << "begin" << nl_string(indent(level))
            << "if B then" << nl_string(indent(indent(level)))
            << "return L3D_1;" << nl_string(indent(level))
            << "else" << nl_string(indent(indent(level)))
            << "return L3D_0;" << nl_string(indent(level))
            << "end if;";
      } else {
         of << "(B : Boolean) return std_logic is" << nl_string(level)
            << "begin" << nl_string(indent(level))
            << "if B then" << nl_string(indent(indent(level)))
            << "return '1';" << nl_string(indent(level))
            << "else" << nl_string(indent(indent(level)))
            << "return '0';" << nl_string(indent(level))
            << "end if;";
      }
      break;
   case SF_UNSIGNED_TO_LOGIC:
      of << "(X : unsigned) return std_logic is" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "return X(0);";
      break;
   case SF_SIGNED_TO_LOGIC:
      of << "(X : signed) return std_logic is" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "return X(0);";
      break;
   case SF_REDUCE_OR:
      emit_reduction(of, level, "or", '0');
      break;
   case SF_REDUCE_AND:
      emit_reduction(of, level, "and", '1');
      break;
   case SF_REDUCE_XOR:
      emit_reduction(of, level, "xor", '0');
      break;
   case SF_REDUCE_XNOR:
      emit_reduction(of, level, "xnor", '0');
      break;
   case SF_TERNARY_LOGIC:
      // sv2vhdl mode operates on logic3d; function_type() already returns
      // logic3d here, so the definition's signature must match (mirrors
      // SF_TERNARY_UNSIGNED below). emit_ternary() is type-agnostic.
      if (get_sv2vhdl_mode())
         of << "(T : Boolean; X, Y : logic3d) return logic3d is";
      else
         of << "(T : Boolean; X, Y : std_logic) return std_logic is";
      emit_ternary(of, level);
      break;
   case SF_TERNARY_SIGNED:
      if (get_sv2vhdl_mode())
         of << "(T : Boolean; X, Y : logic3d_vector) return logic3d_vector is";
      else
         of << "(T : Boolean; X, Y : signed) return signed is";
      emit_ternary(of, level);
      break;
   case SF_TERNARY_UNSIGNED:
      if (get_sv2vhdl_mode())
         of << "(T : Boolean; X, Y : logic3d_vector) return logic3d_vector is";
      else
         of << "(T : Boolean; X, Y : unsigned) return unsigned is";
      emit_ternary(of, level);
      break;
   case SF_LOGIC_TO_INTEGER:
      of << "(X : std_logic) return integer is" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "if X = '1' then return 1; else return 0; end if;";
      break;
   case SF_TIME_FIELD:
      // The field width of %0t, %<N>t and %0<N>t: S is sv_tstr's text,
      // right-justified to the $timeformat width (the formatted time never
      // starts with a blank, so leading blanks are only that padding).
      // Drop the padding, then pad to W -- with zeros if Z, ahead of any
      // sign as vvp does -- never truncating.
      of << "(S : string; W : natural; Z : Boolean) return string is"
         << nl_string(indent(level))
         << "variable F : integer := S'low;" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "while F < S'high and S(F) = ' ' loop"
         << nl_string(indent(indent(level)))
         << "F := F + 1;" << nl_string(indent(level))
         << "end loop;" << nl_string(indent(level))
         << "if S'high - F + 1 >= W then" << nl_string(indent(indent(level)))
         << "return S(F to S'high);" << nl_string(indent(level))
         << "elsif Z then" << nl_string(indent(indent(level)))
         << "return (1 to W - (S'high - F + 1) => '0') & S(F to S'high);"
         << nl_string(indent(level))
         << "else" << nl_string(indent(indent(level)))
         << "return (1 to W - (S'high - F + 1) => ' ') & S(F to S'high);"
         << nl_string(indent(level))
         << "end if;";
      break;
   case SF_REAL_G:
      // vvp's bare real: C's %#g (sys_display.c; %g only under -compatible),
      // 2.5 -> 2.50000, 1e20 -> 1.00000e+20. nvc's to_string(real, fmt)
      // refuses the '#' flag, so as C does it: the exponent X of %.5e (after
      // its rounding) picks %.5e when X < -4 or X >= 6, else %.<5-X>f.
      of << "(R : real) return string is" << nl_string(indent(level))
         << "constant E : string := to_string(R, \"%.5e\");"
         << nl_string(indent(level))
         << "variable P, X : integer;" << nl_string(indent(level))
         << "variable Neg : boolean := false;" << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "P := E'high;" << nl_string(indent(level))
         << "while P > E'low and E(P) /= 'e' loop" << nl_string(indent(indent(level)))
         << "P := P - 1;" << nl_string(indent(level))
         << "end loop;" << nl_string(indent(level))
         << "if E(P) /= 'e' then" << nl_string(indent(indent(level)))
         << "return E;  -- inf, nan" << nl_string(indent(level))
         << "end if;" << nl_string(indent(level))
         << "X := 0;" << nl_string(indent(level))
         << "for K in P + 1 to E'high loop" << nl_string(indent(indent(level)))
         << "if E(K) = '-' then" << nl_string(indent(indent(indent(level))))
         << "Neg := true;" << nl_string(indent(indent(level)))
         << "elsif E(K) >= '0' and E(K) <= '9' then"
         << nl_string(indent(indent(indent(level))))
         << "X := X * 10 + (character'pos(E(K)) - character'pos('0'));"
         << nl_string(indent(indent(level)))
         << "end if;" << nl_string(indent(level))
         << "end loop;" << nl_string(indent(level))
         << "if Neg then" << nl_string(indent(indent(level)))
         << "X := -X;" << nl_string(indent(level))
         << "end if;" << nl_string(indent(level))
         << "if X < -4 or X >= 6 then" << nl_string(indent(indent(level)))
         << "return E;" << nl_string(indent(level))
         << "end if;" << nl_string(indent(level))
         << "return to_string(R, \"%.\" & integer'image(5 - X) & \"f\");";
      break;
   case SF_REM_SIGNED:
      // Verilog's signed %: the remainder takes the DIVIDEND's sign (VHDL
      // rem); a % 0 is all x. (sv2vhdl's l3d_mod_s used VHDL mod before
      // round 6, whose result takes the divisor's sign: -7 % 3 gave 2 where
      // Verilog gives -1.) Value planes only, as l3d_div_s.
      of << "(A, B : logic3d_vector) return logic3d_vector is"
         << nl_string(indent(level))
         << "variable R : logic3d_vector(A'range) := (others => L3D_X);"
         << nl_string(level)
         << "begin" << nl_string(indent(level))
         << "if l3d_to_unsigned(B) = 0 then" << nl_string(indent(indent(level)))
         << "return R;" << nl_string(indent(level))
         << "end if;" << nl_string(indent(level))
         << "return unsigned_to_l3d(unsigned(std_logic_vector("
            "ieee.numeric_std.resize(" << nl_string(indent(indent(level)))
         << "signed(std_logic_vector(l3d_to_unsigned(A))) rem "
            "signed(std_logic_vector(l3d_to_unsigned(B))), A'length))));";
      break;
   default:
      assert(false);
   }

   of << nl_string(level) << "end function;";
}
