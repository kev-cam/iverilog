/*
 *  VHDL code generation for logic devices.
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
#include "vhdl_element.hh"
#include "state.hh"

#include <cassert>
#include <cstring>
#include <sstream>
#include <iostream>

using namespace std;

static const char *drive_names[] = {
   "highz", "small", "medium", "weak", "large", "pull", "strong", "supply"
};

static bool is_sv2vhdl_mode()
{
   return get_sv2vhdl_mode();
}

// Forward declaration
static void default_logic(vhdl_arch *arch, ivl_net_logic_t log);

/*
 * Build a safe VHDL instance name from a base name and prefix.
 */
static string make_inst_name(const char *basename, const char *prefix)
{
   ostringstream ss;
   ss << prefix << "_" << basename;
   string name = ss.str();
   // Replace characters not valid in VHDL identifiers
   for (size_t i = 0; i < name.size(); i++) {
      char c = name[i];
      if (!isalnum(c) && c != '_')
         name[i] = '_';
   }
   if (name[0] == '_') name = "inst" + name;
   if (*name.rbegin() == '_') name += "inst";
   // Can't have two consecutive underscores
   size_t pos = name.find("__");
   while (pos != string::npos) {
      name.replace(pos, 2, "_");
      pos = name.find("__");
   }
   return name;
}

/*
 * Add a strength comment to a statement if drive values are non-standard.
 */
static void add_strength_comment(vhdl_element *stmt, ivl_net_logic_t log)
{
   ivl_drive_t d1 = ivl_logic_drive1(log);
   ivl_drive_t d0 = ivl_logic_drive0(log);
   if (d0 != IVL_DR_STRONG || d1 != IVL_DR_STRONG) {
      ostringstream ss;
      ss << "sv_strength: " << drive_names[d1] << "1 " << drive_names[d0] << "0";
      stmt->set_comment(ss.str());
   }
}

/*
 * Convert the inputs of a logic gate to a binary expression.
 */
static vhdl_expr *inputs_to_expr(vhdl_scope *scope, vhdl_binop_t op,
                                 ivl_net_logic_t log)
{
   // Not always std_logic but this is probably OK since
   // the program has already been type checked
   vhdl_binop_expr *gate =
      new vhdl_binop_expr(op, vhdl_type::std_logic());

   int npins = ivl_logic_pins(log);
   for (int i = 1; i < npins; i++) {
      ivl_nexus_t input = ivl_logic_pin(log, i);

      gate->add_expr(readable_ref(scope, input));
   }

   return gate;
}

/*
 * Convert a gate input to an unary expression.
 */
static vhdl_expr *input_to_expr(vhdl_scope *scope, vhdl_unaryop_t op,
                                ivl_net_logic_t log)
{
   ivl_nexus_t input = ivl_logic_pin(log, 1);
   assert(input);

   vhdl_expr *operand = readable_ref(scope, input);
   return new vhdl_unaryop_expr(op, operand, vhdl_type::std_logic());
}

static void bufif_logic(vhdl_arch *arch, ivl_net_logic_t log, bool if0)
{
   ivl_nexus_t output = ivl_logic_pin(log, 0);
   vhdl_var_ref *lhs = nexus_to_var_ref(arch->get_scope(), output);
   assert(lhs);

   vhdl_expr *val = readable_ref(arch->get_scope(), ivl_logic_pin(log, 1));

   vhdl_expr *sel = readable_ref(arch->get_scope(), ivl_logic_pin(log, 2));

   vhdl_expr *cmp;
   if (ivl_logic_width(log) == 1) {
      vhdl_expr *on = new vhdl_const_bit(if0 ? '0' : '1');
      cmp = new vhdl_binop_expr(sel, VHDL_BINOP_EQ, on, NULL);
   }
   else {
      vhdl_expr *zero = (new vhdl_const_int(0))->cast(sel->get_type());
      vhdl_binop_t op = if0 ? VHDL_BINOP_EQ : VHDL_BINOP_NEQ;
      cmp = new vhdl_binop_expr(sel, op, zero, NULL);
   }

   ivl_signal_t sig = find_signal_named(lhs->get_name(), arch->get_scope());
   char zbit;
   switch (ivl_signal_type(sig)) {
   case IVL_SIT_TRI0:
      zbit = '0';
      break;
   case IVL_SIT_TRI1:
      zbit = '1';
      break;
   case IVL_SIT_TRI:
   default:
      zbit = 'Z';
   }

   vhdl_expr *z = new vhdl_const_bit(zbit);
   if (ivl_logic_width(log) > 1)
      z = new vhdl_bit_spec_expr(NULL, z);
   vhdl_cassign_stmt *cass = new vhdl_cassign_stmt(lhs, z);
   cass->add_condition(val, cmp);

   arch->add_stmt(cass);
}

static void comb_udp_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   ivl_udp_t udp = ivl_logic_udp(log);

   // As with regular case statements, the expression in a
   // `with .. select' statement must be "locally static".
   // This is achieved by first combining the inputs into
   // a temporary

   ostringstream ss;
   ss << ivl_logic_basename(log) << "_Tmp";
   int msb = ivl_udp_nin(udp) - 1;
   const vhdl_type *tmp_type = vhdl_type::std_logic_vector(msb, 0);
   vhdl_signal_decl *tmp_decl = new vhdl_signal_decl(ss.str(), tmp_type);
   arch->get_scope()->add_decl(tmp_decl);

   int nin = ivl_udp_nin(udp);
   vhdl_expr *tmp_rhs;
   if (nin == 1) {
      tmp_rhs = nexus_to_var_ref(arch->get_scope(), ivl_logic_pin(log, 1));
      tmp_rhs = tmp_rhs->cast(tmp_type);
   }
   else
      tmp_rhs = inputs_to_expr(arch->get_scope(), VHDL_BINOP_CONCAT, log);

   ss.str("");
   ss << "Input to " << ivl_logic_basename(log) << " "
      << ivl_udp_name(udp) << " UDP";
   tmp_decl->set_comment(ss.str());

   vhdl_var_ref *tmp_ref =
      new vhdl_var_ref(tmp_decl->get_name().c_str(), NULL);
   arch->add_stmt(new vhdl_cassign_stmt(tmp_ref, tmp_rhs));

   // Now we can implement the UDP as a `with .. select' statement
   // by reading values out of the table
   ivl_nexus_t output_nex = ivl_logic_pin(log, 0);
   vhdl_var_ref *out = nexus_to_var_ref(arch->get_scope(), output_nex);
   vhdl_with_select_stmt *ws =
      new vhdl_with_select_stmt(new vhdl_var_ref(*tmp_ref), out);

   // Ensure the select statement completely covers the input space
   // or some strict VHDL compilers will complain
   ws->add_default(new vhdl_const_bit('X'));

   int nrows = ivl_udp_rows(udp);
   for (int i = 0; i < nrows; i++) {
      const char *row = ivl_udp_row(udp, i);

      vhdl_expr *value = new vhdl_const_bit(row[nin]);
      vhdl_expr *cond = new vhdl_const_bits(row, nin, false);

      ivl_expr_t delay_ex = ivl_logic_delay(log, 1);
      vhdl_expr *delay = NULL;
      if (delay_ex)
         delay = translate_time_expr(delay_ex);

      ws->add_condition(value, cond, delay);
   }

   ss.str("");
   ss << "UDP " << ivl_udp_name(udp);
   ws->set_comment(ss.str());

   arch->add_stmt(ws);
}

static void seq_udp_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   ivl_udp_t udp = ivl_logic_udp(log);

   // These will be translated to a process with a single
   // case statement

   vhdl_process *proc = new vhdl_process(ivl_logic_basename(log));

   ostringstream ss;
   ss << "Generated from UDP " << ivl_udp_name(udp);
   proc->set_comment(ss.str());

   // Create a variable to hold the concatenation of the inputs
   int msb = ivl_udp_nin(udp) - 1;
   const vhdl_type *tmp_type = vhdl_type::std_logic_vector(msb, 0);
   proc->get_scope()->add_decl(new vhdl_var_decl("UDP_Inputs", tmp_type));

   // Concatenate the inputs into a single expression that can be
   // used as the test in a case statement (this can't be inserted
   // directly into the case statement due to the requirement that
   // the test expression be "locally static")
   int nin = ivl_udp_nin(udp);
   vhdl_expr *tmp_rhs = NULL;
   if (nin == 1) {
      vhdl_var_ref *ref =
         nexus_to_var_ref(arch->get_scope(), ivl_logic_pin(log, 1));
      tmp_rhs = ref->cast(tmp_type);
      proc->add_sensitivity(ref->get_name());
   }
   else {
      vhdl_binop_expr *concat = new vhdl_binop_expr(VHDL_BINOP_CONCAT, NULL);

      for (int i = 1; i < nin; i++) {
         vhdl_var_ref *ref =
            nexus_to_var_ref(arch->get_scope(), ivl_logic_pin(log, i));
         concat->add_expr(ref);
         proc->add_sensitivity(ref->get_name());
      }

      tmp_rhs = concat;
   }

   proc->get_container()->add_stmt
      (new vhdl_assign_stmt(new vhdl_var_ref("UDP_Inputs", NULL), tmp_rhs));

   arch->add_stmt(proc);
}

static void udp_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   if (ivl_udp_sequ(ivl_logic_udp(log)))
      seq_udp_logic(arch, log);
   else
      comb_udp_logic(arch, log);
}

static vhdl_expr *translate_logic_inputs(vhdl_scope *scope, ivl_net_logic_t log)
{
   switch (ivl_logic_type(log)) {
   case IVL_LO_NOT:
      return input_to_expr(scope, VHDL_UNARYOP_NOT, log);
   case IVL_LO_AND:
      return inputs_to_expr(scope, VHDL_BINOP_AND, log);
   case IVL_LO_OR:
      return inputs_to_expr(scope, VHDL_BINOP_OR, log);
   case IVL_LO_NAND:
      return inputs_to_expr(scope, VHDL_BINOP_NAND, log);
   case IVL_LO_NOR:
      return inputs_to_expr(scope, VHDL_BINOP_NOR, log);
   case IVL_LO_XOR:
      return inputs_to_expr(scope, VHDL_BINOP_XOR, log);
   case IVL_LO_XNOR:
      return inputs_to_expr(scope, VHDL_BINOP_XNOR, log);
   case IVL_LO_BUF:
   case IVL_LO_BUFT:
   case IVL_LO_BUFZ:
      return nexus_to_var_ref(scope, ivl_logic_pin(log, 1));
   case IVL_LO_PULLUP:
      return new vhdl_const_bit('1');
   case IVL_LO_PULLDOWN:
      return new vhdl_const_bit('0');
   default:
      error("Don't know how to translate logic type = %d to expression",
            ivl_logic_type(log));
      return NULL;
   }
}

/*
 * Emit a sv2vhdl entity instantiation for MOS, tristate, and pull gates.
 * These have fixed port names matching the sv2vhdl library.
 */
static void sv_prim_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   const char *entity_name = NULL;
   bool has_pgate = false;  // CMOS has ngate + pgate

   switch (ivl_logic_type(log)) {
   case IVL_LO_NMOS:     entity_name = "sv_nmos";     break;
   case IVL_LO_PMOS:     entity_name = "sv_pmos";     break;
   case IVL_LO_RNMOS:    entity_name = "sv_rnmos";    break;
   case IVL_LO_RPMOS:    entity_name = "sv_rpmos";    break;
   case IVL_LO_CMOS:     entity_name = "sv_cmos";     has_pgate = true; break;
   case IVL_LO_RCMOS:    entity_name = "sv_rcmos";    has_pgate = true; break;
   case IVL_LO_BUFIF0:   entity_name = "sv_bufif0";   break;
   case IVL_LO_BUFIF1:   entity_name = "sv_bufif1";   break;
   case IVL_LO_NOTIF0:   entity_name = "sv_notif0";   break;
   case IVL_LO_NOTIF1:   entity_name = "sv_notif1";   break;
   case IVL_LO_PULLUP:   entity_name = "sv_pullup";   break;
   case IVL_LO_PULLDOWN: entity_name = "sv_pulldown";  break;
   default:
      error("Unsupported logic type %d for sv_prim_logic", ivl_logic_type(log));
      return;
   }

   string inst_name = make_inst_name(ivl_logic_basename(log), entity_name);

   vhdl_entity_inst *inst = new vhdl_entity_inst(
      inst_name.c_str(), "sv2vhdl", entity_name, "behavioral");

   vhdl_scope *scope = arch->get_scope();

   // Pin 0 is always output (y)
   inst->map_port("y", nexus_to_var_ref(scope, ivl_logic_pin(log, 0)));

   if (ivl_logic_type(log) == IVL_LO_PULLUP
       || ivl_logic_type(log) == IVL_LO_PULLDOWN) {
      // Pull gates: only output port
   }
   else if (has_pgate) {
      // CMOS: pin1=data, pin2=ngate, pin3=pgate
      inst->map_port("data", readable_ref(scope, ivl_logic_pin(log, 1)));
      inst->map_port("ngate", readable_ref(scope, ivl_logic_pin(log, 2)));
      inst->map_port("pgate", readable_ref(scope, ivl_logic_pin(log, 3)));
   }
   else {
      // MOS/tristate: pin1=data, pin2=gate or ctrl
      const char *pin2_name;
      switch (ivl_logic_type(log)) {
      case IVL_LO_BUFIF0: case IVL_LO_BUFIF1:
      case IVL_LO_NOTIF0: case IVL_LO_NOTIF1:
         pin2_name = "ctrl";
         break;
      default:
         pin2_name = "gate";
         break;
      }
      inst->map_port("data", readable_ref(scope, ivl_logic_pin(log, 1)));
      inst->map_port(pin2_name, readable_ref(scope, ivl_logic_pin(log, 2)));
   }

   add_strength_comment(inst, log);
   arch->add_stmt(inst);
}

/*
 * Emit a sv2vhdl entity instantiation for multi-input gates
 * (AND, NAND, OR, NOR, XOR, XNOR) with generic map for input count.
 */
static void sv_multi_input_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   const char *entity_name = NULL;
   switch (ivl_logic_type(log)) {
   case IVL_LO_AND:  entity_name = "sv_and";  break;
   case IVL_LO_NAND: entity_name = "sv_nand"; break;
   case IVL_LO_OR:   entity_name = "sv_or";   break;
   case IVL_LO_NOR:  entity_name = "sv_nor";  break;
   case IVL_LO_XOR:  entity_name = "sv_xor";  break;
   case IVL_LO_XNOR: entity_name = "sv_xnor"; break;
   default: assert(false);
   }

   vhdl_scope *scope = arch->get_scope();
   int npins = ivl_logic_pins(log);
   int ninputs = npins - 1;

   string inst_name = make_inst_name(ivl_logic_basename(log), entity_name);
   vhdl_entity_inst *inst = new vhdl_entity_inst(
      inst_name.c_str(), "sv2vhdl", entity_name, "behavioral");

   inst->map_generic("n", new vhdl_const_int(ninputs));

   // Output: y (pin 0)
   inst->map_port("y", nexus_to_var_ref(scope, ivl_logic_pin(log, 0)));

   // Input: a => concatenation of all input pins
   if (ninputs == 1) {
      // Single input: cast to std_logic_vector(0 to 0) via concatenation
      vhdl_binop_expr *concat =
         new vhdl_binop_expr(VHDL_BINOP_CONCAT, NULL);
      concat->add_expr(readable_ref(scope, ivl_logic_pin(log, 1)));
      inst->map_port("a", concat);
   }
   else {
      vhdl_binop_expr *concat =
         new vhdl_binop_expr(VHDL_BINOP_CONCAT, NULL);
      for (int i = 1; i < npins; i++)
         concat->add_expr(readable_ref(scope, ivl_logic_pin(log, i)));
      inst->map_port("a", concat);
   }

   add_strength_comment(inst, log);
   arch->add_stmt(inst);
}

/*
 * Emit a sv2vhdl entity instantiation for BUF/NOT.
 * These have a single input and potentially multiple outputs.
 */
static void sv_buf_not_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   vhdl_scope *scope = arch->get_scope();
   int npins = ivl_logic_pins(log);
   // For BUF/NOT: pin 0..npins-2 are outputs, pin npins-1 is input
   int noutputs = npins - 1;

   // For single-output BUF/NOT, use inline assignment (avoids vector port mismatch)
   if (noutputs == 1) {
      default_logic(arch, log);
      return;
   }

   // Multi-output: use entity instantiation with temp vector signal
   const char *entity_name = (ivl_logic_type(log) == IVL_LO_NOT)
      ? "sv_not" : "sv_buf";

   string inst_name = make_inst_name(ivl_logic_basename(log), entity_name);

   // Create a temporary std_logic_vector signal for the output
   string tmp_name = inst_name + "_y";
   vhdl_type *vec_type = vhdl_type::std_logic_vector(0, noutputs - 1);
   scope->add_decl(new vhdl_signal_decl(tmp_name.c_str(), vec_type));

   vhdl_entity_inst *inst = new vhdl_entity_inst(
      inst_name.c_str(), "sv2vhdl", entity_name, "behavioral");

   inst->map_generic("n", new vhdl_const_int(noutputs));

   // Input: a (last pin)
   inst->map_port("a", readable_ref(scope, ivl_logic_pin(log, npins - 1)));

   // Output: y (temp vector)
   inst->map_port("y", new vhdl_var_ref(tmp_name.c_str(), vec_type));

   add_strength_comment(inst, log);
   arch->add_stmt(inst);

   // Wire temp vector bits to individual output signals
   for (int i = 0; i < noutputs; i++) {
      vhdl_var_ref *lhs = nexus_to_var_ref(scope, ivl_logic_pin(log, i));
      vhdl_var_ref *rhs = new vhdl_var_ref(tmp_name.c_str(), vhdl_type::std_logic());
      rhs->set_slice(new vhdl_const_int(i), 0);
      arch->add_stmt(new vhdl_cassign_stmt(lhs, rhs));
   }
}

/*
 * Emit a concurrent signal assignment with optional strength comment.
 */
static void default_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   ivl_nexus_t output = ivl_logic_pin(log, 0);
   vhdl_var_ref *lhs = nexus_to_var_ref(arch->get_scope(), output);

   vhdl_expr *rhs = translate_logic_inputs(arch->get_scope(), log);
   vhdl_cassign_stmt *ass = new vhdl_cassign_stmt(lhs, rhs);

   ivl_expr_t delay = ivl_logic_delay(log, 1);
   if (delay)
      ass->set_after(translate_time_expr(delay));

   // Add strength comment for continuous assigns with non-standard strength
   if (ivl_logic_is_cassign(log))
      add_strength_comment(ass, log);

   arch->add_stmt(ass);
}

void draw_logic(vhdl_arch *arch, ivl_net_logic_t log)
{
   bool sv2vhdl = is_sv2vhdl_mode();

   switch (ivl_logic_type(log)) {
   // MOS gates: always use entity instantiation (had no support before)
   case IVL_LO_NMOS:
   case IVL_LO_PMOS:
   case IVL_LO_RNMOS:
   case IVL_LO_RPMOS:
   case IVL_LO_CMOS:
   case IVL_LO_RCMOS:
      sv_prim_logic(arch, log);
      break;

   // NOTIF: always use entity instantiation (had no support before)
   case IVL_LO_NOTIF0:
   case IVL_LO_NOTIF1:
      sv_prim_logic(arch, log);
      break;

   // Tristate buffers
   case IVL_LO_BUFIF0:
      if (sv2vhdl)
         sv_prim_logic(arch, log);
      else
         bufif_logic(arch, log, true);
      break;
   case IVL_LO_BUFIF1:
      if (sv2vhdl)
         sv_prim_logic(arch, log);
      else
         bufif_logic(arch, log, false);
      break;

   // Pull gates
   case IVL_LO_PULLUP:
   case IVL_LO_PULLDOWN:
      if (sv2vhdl)
         sv_prim_logic(arch, log);
      else
         default_logic(arch, log);
      break;

   // Multi-input logic gates
   case IVL_LO_AND:
   case IVL_LO_NAND:
   case IVL_LO_OR:
   case IVL_LO_NOR:
   case IVL_LO_XOR:
   case IVL_LO_XNOR:
      if (sv2vhdl)
         sv_multi_input_logic(arch, log);
      else
         default_logic(arch, log);
      break;

   // BUF/NOT
   case IVL_LO_BUF:
   case IVL_LO_NOT:
      if (sv2vhdl)
         sv_buf_not_logic(arch, log);
      else
         default_logic(arch, log);
      break;

   // Transparent buffers / continuous assigns: always keep as signal assignment
   case IVL_LO_BUFT:
   case IVL_LO_BUFZ:
      default_logic(arch, log);
      break;

   case IVL_LO_UDP:
      udp_logic(arch, log);
      break;

   default:
      error("Don't know how to translate logic type = %d",
            ivl_logic_type(log));
   }
}

/*
 * Emit a sv2vhdl entity instantiation for a single switch.
 */
static void draw_one_switch(vhdl_arch *arch, ivl_switch_t sw)
{
   const char *entity_name = NULL;
   const char *arch_name = "strength";
   bool has_enable = false;

   switch (ivl_switch_type(sw)) {
   case IVL_SW_TRAN:     entity_name = "sv_tran";     break;
   case IVL_SW_TRANIF0:  entity_name = "sv_tranif0";  has_enable = true; break;
   case IVL_SW_TRANIF1:  entity_name = "sv_tranif1";  has_enable = true; break;
   case IVL_SW_RTRAN:    entity_name = "sv_rtran";    break;
   case IVL_SW_RTRANIF0: entity_name = "sv_rtranif0"; has_enable = true; break;
   case IVL_SW_RTRANIF1: entity_name = "sv_rtranif1"; has_enable = true; break;
   case IVL_SW_TRAN_VP:
      debug_msg("Skipping TRAN_VP switch %s (part-select tran not supported)",
                ivl_switch_basename(sw));
      return;
   default:
      error("Unknown switch type %d", ivl_switch_type(sw));
      return;
   }

   string inst_name = make_inst_name(ivl_switch_basename(sw), entity_name);
   vhdl_entity_inst *inst = new vhdl_entity_inst(
      inst_name.c_str(), "sv2vhdl", entity_name, arch_name);

   vhdl_scope *scope = arch->get_scope();

   // Port A
   inst->map_port("a", nexus_to_var_ref(scope, ivl_switch_a(sw)));

   // Port B
   inst->map_port("b", nexus_to_var_ref(scope, ivl_switch_b(sw)));

   // Enable (ctrl) for conditional tran
   if (has_enable) {
      inst->map_port("ctrl", readable_ref(scope, ivl_switch_enable(sw)));
   }

   // Source location comment
   ostringstream ss;
   ss << "Generated from " << entity_name << " at "
      << ivl_switch_file(sw) << ":" << ivl_switch_lineno(sw);
   inst->set_comment(ss.str());

   arch->add_stmt(inst);
}

void draw_switches(vhdl_arch *arch, ivl_scope_t scope)
{
   unsigned nswitches = ivl_scope_switches(scope);
   for (unsigned i = 0; i < nswitches; i++)
      draw_one_switch(arch, ivl_scope_switch(scope, i));
}
