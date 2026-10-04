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
#include <map>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <functional>

using namespace std;

static void emit_wait_for_0(vhdl_procedural *proc, stmt_container *container,
                            ivl_statement_t stmt, vhdl_expr *expr);
static bool deposits_signal(vhdl_procedural *proc, const std::string &name,
                            bool blocking, ivl_signal_t sig = NULL);
static bool number_is_long(ivl_expr_t expr);
static long get_number_as_long(ivl_expr_t expr);
static vhdl_expr *icg2en_pos_term(vhdl_process *proc, ivl_nexus_t gnex,
                                  std::string *sens_name);
bool icg2en_port_mode(ivl_scope_t scope, ivl_nexus_t gnex,
                      std::vector<std::string> *paths, bool use_labels);
// Defined with the pre-statement hook (end of file): a loop test translated
// with the hook's statements going to `pre' (draw_while), and $random /
// $urandom called as a task
static vhdl_expr *translate_loop_test(ivl_expr_t cond, stmt_container *pre);
static int draw_stask_random(vhdl_procedural *proc, stmt_container *container,
                             ivl_statement_t stmt);
static int draw_while_drawn_test(vhdl_procedural *proc,
                                 stmt_container *container,
                                 ivl_statement_t stmt, ivl_statement_t step,
                                 vhdl_expr *test, stmt_container *pre,
                                 vhdl_labeled_loop_stmt *brk,
                                 vhdl_labeled_loop_stmt *cont);

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
/*
 * $finish(0) and $stop(0) print nothing (IEEE 1364 17.4.1; vvp and VCS print
 * their end message for 1 and 2, the default 1), where nvc's std.env.finish
 * and std.env.stop always note "FINISH called" / "STOP called". A note
 * "sv2vhdl: quiet end" just before the call tells the output filters
 * (bin/vvp-sv2ghdl, vamos's OutputFilter) to drop the next one.
 */
static void quiet_end_marker(stmt_container *container, ivl_statement_t stmt)
{
   if (!get_sv2vhdl_mode() || ivl_stmt_parm_count(stmt) < 1)
      return;
   ivl_expr_t level = ivl_stmt_parm(stmt, 0);
   if (level == NULL
       || (ivl_expr_type(level) != IVL_EX_NUMBER
           && ivl_expr_type(level) != IVL_EX_ULONG)
       || !number_is_long(level) || get_number_as_long(level) != 0)
      return;
   container->add_stmt(new vhdl_report_stmt(
      new vhdl_const_string("sv2vhdl: quiet end"), SEVERITY_NOTE));
}

static int draw_stask_finish(vhdl_procedural *, stmt_container *container,
                             ivl_statement_t stmt)
{
   // Emit any $write text still waiting for a newline before ending
   if (get_sv2vhdl_mode())
      container->add_stmt(new vhdl_pcall_stmt("sv_write_flush"));
   quiet_end_marker(container, stmt);

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

/*
 * $stop (sv2vhdl mode): end the run through std.env.stop, the way $finish
 * ends it through std.env.finish. A batch run (vvp -n, a VCS simv without an
 * interactive shell) has nowhere to suspend to, so the stop ends it; nvc exits
 * 0 after "STOP called". The optional diagnostic level argument is ignored.
 */
static int draw_stask_stop(vhdl_procedural *, stmt_container *container,
                           ivl_statement_t stmt)
{
   container->add_stmt(new vhdl_pcall_stmt("sv_write_flush"));
   quiet_end_marker(container, stmt);
   container->add_stmt(new vhdl_pcall_stmt("std.env.stop"));
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

// A comparison result (==, !=, ===, <, &&, ... translate to a VHDL boolean)
// shown by $display is a 1-bit Verilog value: 1 or 0 under every format, as
// vvp prints it -- never "true"/"false", which a boolean's 'image gave (%d,
// %0d, %b, %h and a bare argument all fell back to it). sv2vhdl mode turns it
// into a logic3d bit, which every format below handles.
static vhdl_expr *display_bool_as_bit(vhdl_expr *base)
{
   if (base != NULL && get_sv2vhdl_mode() && base->get_type() != NULL
       && base->get_type()->get_name() == VHDL_TYPE_BOOLEAN)
      return base->cast(vhdl_type::logic3d());
   return base;
}

// A bare argument of a $fdisplayh/b/o-style task (dflt 'h', 'b' or 'o'):
// every bit of it in that radix, as vvp prints it (get_numeric with
// vpiHexStrVal / vpiBinStrVal / vpiOctStrVal).
static vhdl_expr *display_radix_text(vhdl_expr *base, ivl_expr_t e, char dflt)
{
   const vhdl_type *t = base->get_type();
   int w = ivl_expr_width(e);
   if (w < 1)
      w = 1;
   vhdl_expr *v = base;
   if (t != NULL && t->get_name() == VHDL_TYPE_INTEGER) {
      vhdl_type l3(VHDL_TYPE_LOGIC3D_VECTOR, w - 1, 0);
      v = base->cast(&l3);
      t = v->get_type();
   }
   if (t == NULL || (t->get_name() != VHDL_TYPE_LOGIC3D
                     && t->get_name() != VHDL_TYPE_LOGIC3D_VECTOR))
      return base->cast(vhdl_type::string());
   int hi = 0, lo = 0;
   if (t->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      hi = t->get_msb();
      lo = t->get_lsb();
   }
   vhdl_fcall *slv = new vhdl_fcall("to_std_logic_vector",
                                    vhdl_type::std_logic_vector(hi, lo));
   slv->add_expr(v);
   vhdl_fcall *f = new vhdl_fcall(dflt == 'h' ? "sv_hstr"
                                  : dflt == 'b' ? "sv_bstr" : "sv_ostr",
                                  vhdl_type::string());
   f->add_expr(slv);
   return f;
}

/*
 * %m and the "Scope:" line of $error / $fatal / ...: the Verilog name of the
 * scope the statement is in. In a module instantiated once that is a
 * constant. A module instantiated more than once is one VHDL entity, drawn
 * once, so a constant named the first instance in every instance (`sub u1(),
 * u2();' printed tb.u1 twice, where vvp prints tb.u1 and tb.u2): there the
 * name is the architecture's constant SV_Hier_Name, the instance's own name,
 * which SV_Hier_Lookup finds from the entity's 'PATH_NAME at elaboration
 * (declare_hier_names), followed by the scope's path inside the module (a
 * named block, a task).
 */
static std::map<vhdl_entity*, ivl_scope_t> g_hier_name_entities;

vhdl_expr *hier_name_expr()
{
   const std::string name = active_hier_name();
   ivl_scope_t m = get_active_scope();
   while (m != NULL && ivl_scope_type(m) != IVL_SCT_MODULE)
      m = ivl_scope_parent(m);
   if (!get_sv2vhdl_mode() || m == NULL)
      return new vhdl_const_string(name);
   std::vector<ivl_scope_t> insts;
   same_type_instances(m, insts);
   vhdl_entity *ent = find_entity(m);
   const std::string mname = ivl_scope_name(m);
   if (insts.size() < 2 || ent == NULL || ent->get_arch() == NULL
       || name.compare(0, mname.size(), mname) != 0)
      return new vhdl_const_string(name);
   if (!g_hier_name_entities.count(ent))
      g_hier_name_entities[ent] = m;
   vhdl_var_ref *inst = new vhdl_var_ref("SV_Hier_Name", vhdl_type::string());
   const std::string inside = name.substr(mname.size());
   if (inside.empty())
      return inst;
   vhdl_binop_expr *cat = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
                                              vhdl_type::string());
   cat->add_expr(inst);
   cat->add_expr(new vhdl_const_string(inside));
   return cat;
}

// A VHDL string literal of `s'
static std::string vhdl_string_literal(const std::string &s)
{
   std::string out = "\"";
   for (size_t i = 0; i < s.size(); i++) {
      if (s[i] == '"')
         out += "\"\"";
      else if ((unsigned char)s[i] < ' ' || (unsigned char)s[i] > '~')
         out += "?";
      else
         out += s[i];
   }
   return out + "\"";
}

/*
 * In each architecture hier_name_expr used: (ahead of its other
 * declarations, as a Verilog function drawn there may use it)
 *
 *    -- %m: this instance's Verilog name (sub has 2 instances)
 *    function SV_Hier_Lookup(P : string) return string is
 *    begin
 *       if P'length >= 18 and P(P'high - 17 to P'high) = ":a:u2:sv_hier_mark" then
 *          return "tb.a.u2";
 *       end if;
 *       ...
 *       return "tb.u1";
 *    end function;
 *    constant SV_Hier_Mark : boolean := true;
 *    constant SV_Hier_Name : string := SV_Hier_Lookup(SV_Hier_Mark'path_name);
 *
 * P is the path of the architecture's own constant SV_Hier_Mark
 * (":tb:a:u2:sv_hier_mark"; the entity's name is not used: sv-rename-variants
 * renames an entity with variants); each instance is known by the labels
 * from its design root down to it (instance_vhdl_path), tried longest first,
 * so a wrapper above the root (vamos's vamos_tops) changes nothing.
 */
void declare_hier_names()
{
   for (std::map<vhdl_entity*, ivl_scope_t>::iterator it =
           g_hier_name_entities.begin();
        it != g_hier_name_entities.end(); ++it) {
      vhdl_entity *ent = it->first;
      std::vector<ivl_scope_t> insts;
      same_type_instances(it->second, insts);
      std::vector<std::pair<std::string, std::string> > table;
      for (size_t k = 0; k < insts.size(); k++) {
         std::string path;
         if (instance_vhdl_path(insts[k], path) && path != ":")
            table.push_back(std::make_pair(path + "sv_hier_mark",
                                           ivl_scope_name(insts[k])));
      }
      std::stable_sort(table.begin(), table.end(),
                       [](const std::pair<std::string, std::string> &a,
                          const std::pair<std::string, std::string> &b) {
                          return a.first.size() > b.first.size();
                       });
      std::vector<std::string> fn;
      ostringstream hd;
      hd << "-- %m: this instance's Verilog name (" << ivl_scope_tname(it->second)
         << " has " << insts.size() << " instances)";
      fn.push_back(hd.str());
      fn.push_back("function SV_Hier_Lookup(P : string) return string is");
      fn.push_back("begin");
      for (size_t k = 0; k < table.size(); k++) {
         const size_t n = table[k].first.size();
         ostringstream ln;
         ln << "  if P'length >= " << n << " and P(P'high - " << n - 1
            << " to P'high) = " << vhdl_string_literal(table[k].first)
            << " then return " << vhdl_string_literal(table[k].second)
            << "; end if;";
         fn.push_back(ln.str());
      }
      fn.push_back("  return " + vhdl_string_literal(ivl_scope_name(it->second))
                   + ";");
      fn.push_back("end function;");
      vhdl_scope *as = ent->get_arch()->get_scope();
      std::vector<std::string> mk, cn;
      mk.push_back("constant SV_Hier_Mark : boolean := true;");
      cn.push_back("constant SV_Hier_Name : string := "
                   "SV_Hier_Lookup(SV_Hier_Mark'path_name);");
      // (in front of the other declarations, in this order: the lookup, the
      // mark, the name)
      as->add_forward_decl(new vhdl_verbatim_decl("SV_Hier_Name", cn));
      as->add_forward_decl(new vhdl_verbatim_decl("SV_Hier_Mark", mk));
      as->add_forward_decl(new vhdl_verbatim_decl("SV_Hier_Lookup", fn));
   }
   g_hier_name_entities.clear();
}

// Build the concatenated display text for a $display-family statement's
// parameters. `proc' may be NULL when building the body of a companion
// (postponed) $monitor/$strobe process -- no wait-for-0 statements are needed
// there, and none can be emitted. Returns NULL on translation failure.
// dflt: the format of an argument with no format of its own -- 'd' (the
// $display default), or 'h', 'b', 'o' for the radix forms of the file tasks
// ($fdisplayh, $fwriteb, ... in fileio.cc).
vhdl_expr *build_display_text(vhdl_procedural *proc,
                              stmt_container *container,
                              ivl_statement_t stmt,
                              int first_parm = 0, char dflt = 'd')
{
   vhdl_binop_expr *text = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
                                               vhdl_type::string());

   const int count = ivl_stmt_parm_count(stmt);
   int i = first_parm;
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
                  // Embedded newline: VHDL string literals cannot hold
                  // LF, so flush and concatenate the LF character
                  // (multi_bit_strength gold needs the line break)
                  text->add_expr(new vhdl_const_string(ss.str()));
                  ss.str("");
                  text->add_expr(new vhdl_var_ref("LF",
                                                  vhdl_type::string()));
               }
               else
                  ss << ch;   // NULs dropped later, by vhdl_const_string::emit
               p += 3;
            }
            else if (*p == '%' && *(++p) != '%') {
               // Flush the output string up to this point
               text->add_expr(new vhdl_const_string(ss.str()));
               ss.str("");

               // Parse the Verilog field-width spec: %[0][N]. A leading '0'
               // selects minimum width / no padding (%0d); explicit digits N
               // give the field width; plain %d uses a size-derived default
               // (applied below in the decimal branch).
               bool ld_zero = false;
               long fw_spec = -1;   // -1 => no explicit width digits
               if (*p == '0') { ld_zero = true; ++p; }
               {
                  bool has_digits = false; long wv = 0;
                  while (isdigit(*p)) { has_digits = true; wv = wv*10 + (*p - '0'); ++p; }
                  if (has_digits) fw_spec = wv;
               }
               // Optional .precision (real formats: %5.2f etc.)
               long prec_spec = -1;
               if (*p == '.') {
                  ++p;
                  bool has_digits = false; long pv = 0;
                  while (isdigit(*p)) { has_digits = true; pv = pv*10 + (*p - '0'); ++p; }
                  if (has_digits) prec_spec = pv;
               }

               switch (*p) {
               case 'm':
                  // %m = the hierarchical name of the scope containing this
                  // $display, from the scope-keyed store (set in draw_process,
                  // and for a function, task or named block in its drawing).
                  text->add_expr(hier_name_expr());
                  break;
               case 't': case 'T':
                  {
                     // %t: format a time value per the current $timeformat.
                     // sv_tstr(v, vu, dflt) takes a time v in units of
                     // 10^vu s (the scope's time units) and scales it to the
                     // $timeformat units, or when $timeformat was never
                     // called to dflt: the smallest precision of the whole
                     // DESIGN (IEEE 1364 17.3.2, vvp), not this scope's.
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i++);
                     assert(netp);
                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return NULL;
                     emit_wait_for_0(proc, container, stmt, base);
                     base = display_bool_as_bit(base);
                     const bool real_arg = ivl_expr_value(netp) == IVL_VT_REAL
                        || (base->get_type() != NULL
                            && base->get_type()->get_name() == VHDL_TYPE_REAL);
                     // A real argument ($realtime, a real variable) is a time
                     // in scope units WITH a fraction (5.355 for 5.355 ns): it
                     // goes to sv_tstr's REAL overload, which scales it in
                     // double precision and prints it with "%.<p>f" as vvp's
                     // get_time_real does -- every digit the $timeformat asks
                     // for, and no INTEGER range limit (an integer() cast of
                     // the count stopped the run past 2^31 precision ticks,
                     // 2.147 ms at 1 ps). Any other argument is an integer
                     // count: sv_tstr's integer overload scales its digit
                     // string, so a 64-bit $time quotient is exact too.
                     vhdl_type rtype(VHDL_TYPE_REAL);
                     vhdl_type itype(VHDL_TYPE_INTEGER);
                     vhdl_fcall *f = new vhdl_fcall("sv_tstr",
                                                    vhdl_type::string());
                     f->add_expr(base->cast(real_arg ? &rtype : &itype));
                     f->add_expr(new vhdl_const_int(active_time_units()));
                     f->add_expr(new vhdl_const_int(
                        ivl_design_time_precision(get_vhdl_design())));
                     if (prec_spec >= 0) {
                        // sv_tstr always uses the $timeformat precision
                        cerr << "Warning: " << ivl_stmt_file(stmt) << ":"
                             << ivl_stmt_lineno(stmt) << ": the precision of %"
                             << (ld_zero ? "0" : "");
                        if (fw_spec >= 0)
                           cerr << fw_spec;
                        cerr << "." << prec_spec << *p << " is not translated:"
                             << " the $timeformat precision is used" << endl;
                     }
                     if (ld_zero || fw_spec >= 0) {
                        // A field width other than the $timeformat one
                        // (vvp, VCS): %0t has none (no padding), %<N>t pads
                        // to N blanks and %0<N>t to N zeros.
                        require_support_function(SF_TIME_FIELD);
                        vhdl_fcall *w = new vhdl_fcall(
                           support_function::function_name(SF_TIME_FIELD),
                           vhdl_type::string());
                        w->add_expr(f);
                        w->add_expr(new vhdl_const_int(
                           fw_spec >= 0 ? (int)fw_spec : 0));
                        w->add_expr(new vhdl_const_bool(ld_zero
                                                        && fw_spec >= 0));
                        text->add_expr(w);
                     }
                     else
                        text->add_expr(f);
                  }
                  break;
               case 'f': case 'F': case 'g': case 'G': case 'e':
                  {
                     // Real formats. Verilog %f/%g/%e are C printf semantics,
                     // and VHDL-2008 to_string(real, fmt) is implemented with
                     // C snprintf in nvc -- so reconstruct the C format string
                     // and pass it through verbatim.
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i++);
                     assert(netp);
                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return NULL;
                     emit_wait_for_0(proc, container, stmt, base);
                     base = display_bool_as_bit(base);
                     ostringstream fs;
                     fs << '%';
                     if (ld_zero) fs << '0';
                     if (fw_spec >= 0) fs << fw_spec;
                     if (prec_spec >= 0) fs << '.' << prec_spec;
                     fs << (char)tolower(*p);
                     vhdl_type rt(VHDL_TYPE_REAL);
                     vhdl_fcall *f = new vhdl_fcall("to_string",
                                                    vhdl_type::string());
                     f->add_expr(base->cast(&rt));
                     f->add_expr(new vhdl_const_string(fs.str()));
                     text->add_expr(f);
                  }
                  break;
               case 'v': case 'V':
                  {
                     // %v: value with drive strength.  Scalar logic3d
                     // signals, vector logic3d signals (per-bit,
                     // MSB-first, '_'-joined) and constant bit-selects
                     // query the kernel net solver via sv_vstr, with a
                     // value-alphabet fallback for non-kernel nets.
                     // Every other %v shape keeps the historical
                     // default handling.
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i);
                     ivl_signal_t vsig = NULL;
                     long sel_idx = -1;
                     if (netp != NULL
                         && ivl_expr_type(netp) == IVL_EX_SIGNAL)
                        vsig = ivl_expr_signal(netp);
                     else if (netp != NULL
                              && ivl_expr_type(netp) == IVL_EX_SELECT
                              && ivl_expr_width(netp) == 1) {
                        // Constant bit-select of a signal: the kernel
                        // net key is the indexed actual, e.g. w(2)
                        ivl_expr_t bse = ivl_expr_oper1(netp);
                        ivl_expr_t off = ivl_expr_oper2(netp);
                        if (bse != NULL && off != NULL
                            && ivl_expr_type(bse) == IVL_EX_SIGNAL
                            && (ivl_expr_type(off) == IVL_EX_NUMBER
                                || ivl_expr_type(off) == IVL_EX_ULONG)
                            && number_is_long(off)) {
                           vsig = ivl_expr_signal(bse);
                           sel_idx = get_number_as_long(off);
                        }
                     }
                     if (vsig == NULL)
                        goto default_fmt;

                     i++;
                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return NULL;
                     emit_wait_for_0(proc, container, stmt, base);
                     base = display_bool_as_bit(base);
                     const vhdl_type *bt = base->get_type();
                     const vhdl_type_name_t btn =
                        bt == NULL ? VHDL_TYPE_STD_LOGIC : bt->get_name();
                     if (btn != VHDL_TYPE_LOGIC3D
                         && btn != VHDL_TYPE_LOGIC3D_VECTOR) {
                        // Consumed but not a logic3d shape: plain
                        // 4-state characters
                        vhdl_fcall *f = new vhdl_fcall("sv_bstr",
                                                       vhdl_type::string());
                        vhdl_fcall *conv = new vhdl_fcall("to_std_logic_vector",
                           vhdl_type::std_logic_vector(0, 0));
                        conv->add_expr(base);
                        f->add_expr(conv);
                        text->add_expr(f);
                        break;
                     }

                     string vpath =
                        string(ivl_scope_name(ivl_signal_scope(vsig)))
                        + "." + ivl_signal_basename(vsig);
                     if (sel_idx >= 0) {
                        ostringstream sfx;
                        sfx << "(" << sel_idx << ")";
                        vpath += sfx.str();
                     }
                     for (size_t k = 0; k < vpath.size(); k++)
                        vpath[k] = tolower(vpath[k]);
                     vhdl_fcall *f = new vhdl_fcall("sv_vstr",
                                                    vhdl_type::string());
                     f->add_expr(base);
                     f->add_expr(new vhdl_const_string(vpath.c_str()));
                     text->add_expr(f);
                  }
                  break;
               case 'c': case 'C':
                  {
                     // %c: the argument's low 8 bits as one ASCII
                     // character (historically fell into the default
                     // 'image path, printing logic3d tuples)
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i++);
                     assert(netp);
                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return NULL;
                     emit_wait_for_0(proc, container, stmt, base);
                     base = display_bool_as_bit(base);
                     const vhdl_type *bt = base->get_type();
                     const vhdl_type_name_t btn = bt == NULL
                        ? VHDL_TYPE_INTEGER : bt->get_name();
                     vhdl_expr *slv = NULL;
                     if (btn == VHDL_TYPE_LOGIC3D_VECTOR) {
                        vhdl_fcall *conv = new vhdl_fcall("to_std_logic_vector",
                           vhdl_type::std_logic_vector(bt->get_msb(),
                                                       bt->get_lsb()));
                        conv->add_expr(base);
                        slv = conv;
                     } else if (btn == VHDL_TYPE_LOGIC3D) {
                        vhdl_fcall *conv = new vhdl_fcall("to_std_logic_vector",
                           vhdl_type::std_logic_vector(0, 0));
                        conv->add_expr(base);
                        slv = conv;
                     } else if ((btn == VHDL_TYPE_UNSIGNED
                                 || btn == VHDL_TYPE_SIGNED)
                                && !base->constant()) {
                        vhdl_fcall *conv = new vhdl_fcall("std_logic_vector",
                           vhdl_type::std_logic_vector(bt->get_msb(),
                                                       bt->get_lsb()));
                        conv->add_expr(base);
                        slv = conv;
                     } else if (btn == VHDL_TYPE_STD_LOGIC_VECTOR) {
                        slv = base;
                     } else {
                        // Integer-typed (character literal or sized
                        // constant): build the 8-bit vector directly
                        vhdl_type itype(VHDL_TYPE_INTEGER);
                        vhdl_fcall *num = new vhdl_fcall("to_unsigned",
                           vhdl_type::nunsigned(8));
                        num->add_expr(base->cast(&itype));
                        num->add_expr(new vhdl_const_int(8));
                        vhdl_fcall *conv = new vhdl_fcall("std_logic_vector",
                           vhdl_type::std_logic_vector(7, 0));
                        conv->add_expr(num);
                        slv = conv;
                     }
                     vhdl_fcall *f = new vhdl_fcall("sv_cstr",
                                                    vhdl_type::string());
                     f->add_expr(slv);
                     text->add_expr(f);
                  }
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
                        return NULL;

                     emit_wait_for_0(proc, container, stmt, base);
                     base = display_bool_as_bit(base);

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
                        } else if (tn == VHDL_TYPE_LOGIC3D_VECTOR) {
                           // sv2vhdl mode: a logic3d_vector formatted with %x/
                           // %h/%b/%o must render each bit's 4-state char (x/z
                           // preserved), not printed via logic3d_vector'image (a
                           // "(2,3,..)" tuple). Convert logic3d_vector ->
                           // std_logic_vector via to_std_logic_vector (per-bit
                           // to_std_logic, certainty preserving) so sv_hstr/
                           // sv_bstr/sv_ostr can emit 0/1/x/z. (Note:
                           // l3d_to_unsigned would drop x/z to value bits.)
                           vhdl_fcall *conv = new vhdl_fcall("to_std_logic_vector",
                              vhdl_type::std_logic_vector(
                                 base->get_type()->get_msb(),
                                 base->get_type()->get_lsb()));
                           conv->add_expr(base);
                           base = conv;
                        } else if (tn == VHDL_TYPE_LOGIC3D) {
                           // sv2vhdl mode: a scalar logic3d formatted with %x/
                           // %h/%b/%o. Convert to a 1-element std_logic_vector
                           // (via to_std_logic_vector) so sv_bstr/sv_hstr renders
                           // one 4-state char, instead of emitting the raw
                           // logic3d integer code via 'image (e.g. "3" for 1).
                           vhdl_fcall *conv = new vhdl_fcall("to_std_logic_vector",
                              vhdl_type::std_logic_vector(0, 0));
                           conv->add_expr(base);
                           base = conv;
                        } else if (tn == VHDL_TYPE_STD_LOGIC_VECTOR
                                   && !base->constant()) {
                           // Already std_logic_vector: use directly
                        } else if (tn == VHDL_TYPE_INTEGER
                                   || ((tn == VHDL_TYPE_UNSIGNED
                                        || tn == VHDL_TYPE_SIGNED)
                                       && base->constant())) {
                           // A sized integer / vector constant reaches us as a
                           // vhdl_const_int (e.g. 16'd3 is typed UNSIGNED but
                           // holds an integer literal, so std_logic_vector() of
                           // it would be illegal). Verilog formats %h/%x/%b/%o
                           // as the radix, NEVER decimal, so rebuild a
                           // width-correct std_logic_vector via to_unsigned/
                           // to_signed and let the radix formatter run, instead
                           // of falling through to integer'image (which printed
                           // decimal -- %h of 16'd3 wrongly gave "3" not "0003").
                           int w = ivl_expr_width(netp);
                           if (w < 1) w = 1;
                           vhdl_type itype(VHDL_TYPE_INTEGER);
                           const bool sgn = ivl_expr_signed(netp) != 0;
                           vhdl_fcall *num = new vhdl_fcall(
                              sgn ? "to_signed" : "to_unsigned",
                              sgn ? vhdl_type::nsigned(w)
                                  : vhdl_type::nunsigned(w));
                           num->add_expr(base->cast(&itype));
                           num->add_expr(new vhdl_const_int(w));
                           vhdl_fcall *slv = new vhdl_fcall("std_logic_vector",
                              vhdl_type::std_logic_vector(w - 1, 0));
                           slv->add_expr(num);
                           base = slv;
                        } else {
                           // Single-bit, real, etc.: fall back
                           text->add_expr(base->cast(text->get_type()));
                           break;
                        }
                     }

                     vhdl_fcall *f = new vhdl_fcall(func,
                                                    vhdl_type::string());
                     f->add_expr(base);
                     if (ld_zero && fw_spec < 0) {
                        // %0b/%0h/%0o (no explicit width): suppress leading
                        // zeros (minimum width). %0Nh is zero-PAD to width N,
                        // not a strip, so it must keep the full rendering.
                        vhdl_fcall *strip = new vhdl_fcall("sv_strip0",
                                                           vhdl_type::string());
                        strip->add_expr(f);
                        text->add_expr(strip);
                     } else
                        text->add_expr(f);
                  }
                  break;
               default:
               default_fmt:
                  {
                     assert(i < count);
                     ivl_expr_t netp = ivl_stmt_parm(stmt, i++);
                     assert(netp);

                     vhdl_expr *base = translate_expr(netp);
                     if (NULL == base)
                        return NULL;

                     emit_wait_for_0(proc, container, stmt, base);
                     base = display_bool_as_bit(base);

                     // Verilog %d with a real argument rounds to the nearest
                     // integer first.
                     if ((*p == 'd' || *p == 'D') && base->get_type()
                         && base->get_type()->get_name() == VHDL_TYPE_REAL) {
                        vhdl_type itype(VHDL_TYPE_INTEGER);
                        base = base->cast(&itype);
                     }

                     // sv2vhdl mode: %d of a logic3d value must print "x" if any
                     // bit is unknown (Verilog %d convention), not the raw
                     // logic3d aggregate "(2,3,..)". Route through sv_dstr on the
                     // 4-state std_logic_vector. Only for 'd'/'D' — %s/%c/etc.
                     // keep their normal handling.
                     bool l3d_dec = false;
                     if ((*p == 'd' || *p == 'D') && base->get_type()) {
                        vhdl_type_name_t tn = base->get_type()->get_name();
                        if (tn == VHDL_TYPE_LOGIC3D_VECTOR
                            || tn == VHDL_TYPE_LOGIC3D) {
                           int hi = 0, lo = 0;
                           if (tn == VHDL_TYPE_LOGIC3D_VECTOR) {
                              hi = base->get_type()->get_msb();
                              lo = base->get_type()->get_lsb();
                           }
                           // Verilog %d field width: %Nd -> N, %0d -> 0 (no
                           // pad), plain %d -> operand max-magnitude width.
                           long field_w;
                           if (fw_spec >= 0) field_w = fw_spec;
                           else if (ld_zero) field_w = 0;
                           else {
                              int w = ivl_expr_width(netp); if (w < 1) w = 1;
                              const bool sgn = ivl_expr_signed(netp) != 0;
                              int magbits = sgn ? w - 1 : w;
                              if (magbits < 0) magbits = 0;
                              if (magbits >= 64) field_w = 20;
                              else {
                                 unsigned long long mv =
                                    magbits ? ((1ULL << magbits) - 1ULL) : 0ULL;
                                 field_w = 1;
                                 while (mv >= 10) { mv /= 10; field_w++; }
                              }
                              if (sgn) field_w += 1;
                           }
                           vhdl_fcall *conv = new vhdl_fcall(
                              "to_std_logic_vector",
                              vhdl_type::std_logic_vector(hi, lo));
                           conv->add_expr(base);
                           vhdl_fcall *f = new vhdl_fcall(
                              ivl_expr_signed(netp) ? "sv_dstr_signed"
                                                    : "sv_dstr",
                              vhdl_type::string());
                           f->add_expr(conv);
                           f->add_expr(new vhdl_const_int((int)field_w));
                           text->add_expr(f);
                           l3d_dec = true;
                        }
                     }
                     // sv2vhdl mode: %s of a logic3d value renders the packed
                     // 8-bit ASCII (Verilog %s), not the raw aggregate image.
                     if (!l3d_dec && (*p == 's' || *p == 'S')
                         && base->get_type()) {
                        vhdl_type_name_t tn = base->get_type()->get_name();
                        if (tn == VHDL_TYPE_LOGIC3D_VECTOR
                            || tn == VHDL_TYPE_LOGIC3D) {
                           int hi = 0, lo = 0;
                           if (tn == VHDL_TYPE_LOGIC3D_VECTOR) {
                              hi = base->get_type()->get_msb();
                              lo = base->get_type()->get_lsb();
                           }
                           vhdl_fcall *conv = new vhdl_fcall(
                              "to_std_logic_vector",
                              vhdl_type::std_logic_vector(hi, lo));
                           conv->add_expr(base);
                           vhdl_fcall *f = new vhdl_fcall("sv_sstr",
                                                          vhdl_type::string());
                           f->add_expr(conv);
                           text->add_expr(f);
                           l3d_dec = true;
                        }
                     }
                     if (!l3d_dec)
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
            return NULL;

         emit_wait_for_0(proc, container, stmt, base);
         base = display_bool_as_bit(base);

         // A bare time function prints as vvp's get_display prints it
         // (vpi/sys_display.c): $time and $simtime right-aligned in 20
         // columns, $stime in 10 (`$monitor($time, ...)' printed "0", vvp
         // "                   0"), $realtime with the digits of its scope's
         // precision (%.<units - precision>f: 3.250 at 1ns/1ps)
         const char *sfname = ivl_expr_type(net) == IVL_EX_SFUNC
            ? ivl_expr_name(net) : "";
         const int time_cols = strcmp(sfname, "$time") == 0
            || strcmp(sfname, "$simtime") == 0 ? 20
            : strcmp(sfname, "$stime") == 0 ? 10 : 0;
         if (time_cols > 0 && base->get_type()
             && base->get_type()->get_name() == VHDL_TYPE_INTEGER) {
            require_support_function(SF_TIME_FIELD);
            vhdl_fcall *w = new vhdl_fcall(
               support_function::function_name(SF_TIME_FIELD),
               vhdl_type::string());
            w->add_expr(base->cast(text->get_type()));
            w->add_expr(new vhdl_const_int(time_cols));
            w->add_expr(new vhdl_const_bool(false));
            text->add_expr(w);
            continue;
         }

         // A bare REAL $display arg formats as vvp formats it: %#g (5.00000,
         // 1.00000e+20; %g only under vvp -compatible), and $realtime as
         // above
         const vhdl_type *bt0 = base->get_type();
         if (bt0 && bt0->get_name() == VHDL_TYPE_REAL) {
            if (strcmp(sfname, "$realtime") == 0) {
               vhdl_fcall *f = new vhdl_fcall("to_string", vhdl_type::string());
               f->add_expr(base);
               const int digits = active_time_units()
                  - active_time_precision();
               ostringstream rs;
               rs << "%." << (digits > 0 ? digits : 0) << "f";
               f->add_expr(new vhdl_const_string(rs.str()));
               text->add_expr(f);
            }
            else {
               require_support_function(SF_REAL_G);
               vhdl_fcall *f = new vhdl_fcall(
                  support_function::function_name(SF_REAL_G),
                  vhdl_type::string());
               f->add_expr(base);
               text->add_expr(f);
            }
            continue;
         }
         if (dflt != 'd') {
            text->add_expr(display_radix_text(base, net, dflt));
            continue;
         }

         // sv2vhdl: a bare $display arg (no format code) uses Verilog's
         // DEFAULT format = DECIMAL (%d), right-justified to the operand's
         // max-magnitude width. A logic3d/logic3d_vector must go through
         // sv_dstr(to_std_logic_vector(...)); base->cast(string) would emit
         // logic3d_vector'image -> raw aggregate "(2,2,2,3)" or a scalar code.
         const vhdl_type *bt = base->get_type();
         if (bt && (bt->get_name() == VHDL_TYPE_LOGIC3D_VECTOR
                    || bt->get_name() == VHDL_TYPE_LOGIC3D)) {
            int hi = 0, lo = 0;
            if (bt->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
               hi = bt->get_msb(); lo = bt->get_lsb();
            }
            int w = ivl_expr_width(net); if (w < 1) w = 1;
            const bool sgn = ivl_expr_signed(net) != 0;
            int magbits = sgn ? w - 1 : w; if (magbits < 0) magbits = 0;
            int fw;
            if (magbits >= 64) fw = 20;
            else {
               unsigned long long mv =
                  magbits ? ((1ULL << magbits) - 1ULL) : 0ULL;
               fw = 1;
               while (mv >= 10) { mv /= 10; fw++; }
            }
            if (sgn) fw += 1;
            vhdl_fcall *conv = new vhdl_fcall("to_std_logic_vector",
               vhdl_type::std_logic_vector(hi, lo));
            conv->add_expr(base);
            vhdl_fcall *f = new vhdl_fcall(
               sgn ? "sv_dstr_signed" : "sv_dstr", vhdl_type::string());
            f->add_expr(conv);
            f->add_expr(new vhdl_const_int(fw));
            text->add_expr(f);
         }
         else
            text->add_expr(base->cast(text->get_type()));
      }
   }

   if (count == 0)
      text->add_expr(new vhdl_const_string(""));

   return text;
}

// Generate VHDL report statements for Verilog $display/$write
static int draw_stask_display(vhdl_procedural *proc,
                              stmt_container *container,
                              ivl_statement_t stmt,
                              bool newline)
{
   vhdl_expr *text = build_display_text(proc, container, stmt);
   if (NULL == text)
      return 1;
   // vvp line semantics: $write text accumulates in the shared line
   // buffer until a newline arrives; $display completes the pending
   // line.  Both print through report inside sv_display_pkg.
   vhdl_pcall_stmt *pc =
      new vhdl_pcall_stmt(newline ? "sv_display_line" : "sv_write_buf");
   pc->add_expr(text);
   container->add_stmt(pc);
   return 0;
}

vhdl_expr *translate_sfunc_simtime(ivl_expr_t);

/*
 * $fatal, $error, $warning and $info (sv2vhdl mode). The message is printed
 * through the $display machinery in exactly the two lines vvp prints
 *
 *   ERROR: <file>:<line>: <message>
 *          Time: <ticks>  Scope: <scope>
 *
 * (the second line indented by the severity word's width plus two; <ticks>
 * is the time in simulation-precision ticks, <scope> the %m name), then a
 * report statement carries the severity into the VHDL run:
 *
 *   $info    -> report "INFO" severity note       (the run continues)
 *   $warning -> report "WARNING" severity warning (the run continues)
 *   $error   -> report "ERROR" severity error     (the run continues; nvc
 *               then exits non-zero at the end of the run)
 *   $fatal   -> report "FATAL" severity failure   (nvc stops, exit non-zero)
 *
 * $fatal's first argument is the finish number (0, 1 or 2), which only sets
 * the verbosity of vvp's $finish and is not printed; a first argument that is
 * a string literal is taken as the message (VCS accepts `$fatal("...")').
 */
static int draw_stask_severity(vhdl_procedural *proc,
                               stmt_container *container,
                               ivl_statement_t stmt)
{
   const char *name = ivl_stmt_name(stmt);
   const char *word;
   vhdl_severity_t level;
   if (strcmp(name, "$fatal") == 0) {
      word = "FATAL"; level = SEVERITY_FAILURE;
   } else if (strcmp(name, "$error") == 0) {
      word = "ERROR"; level = SEVERITY_ERROR;
   } else if (strcmp(name, "$warning") == 0) {
      word = "WARNING"; level = SEVERITY_WARNING;
   } else {
      word = "INFO"; level = SEVERITY_NOTE;
   }

   const int count = ivl_stmt_parm_count(stmt);
   int first = 0;
   if (level == SEVERITY_FAILURE && count > 0) {
      ivl_expr_t p0 = ivl_stmt_parm(stmt, 0);
      if (p0 == NULL || ivl_expr_type(p0) != IVL_EX_STRING)
         first = 1;   // the finish number
   }

   ostringstream head;
   head << word << ": " << ivl_stmt_file(stmt) << ":"
        << ivl_stmt_lineno(stmt) << ": ";
   vhdl_binop_expr *line1 = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
                                                vhdl_type::string());
   line1->add_expr(new vhdl_const_string(head.str()));
   if (first < count) {
      vhdl_expr *msg = build_display_text(proc, container, stmt, first);
      if (NULL == msg)
         return 1;
      line1->add_expr(msg);
   }
   vhdl_pcall_stmt *pc1 = new vhdl_pcall_stmt("sv_display_line");
   pc1->add_expr(line1);
   container->add_stmt(pc1);

   string pad(strlen(word) + 2, ' ');
   vhdl_binop_expr *line2 = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
                                                vhdl_type::string());
   line2->add_expr(new vhdl_const_string(pad + "Time: "));
   line2->add_expr(translate_sfunc_simtime(NULL)->cast(vhdl_type::string()));
   line2->add_expr(new vhdl_const_string("  Scope: "));
   line2->add_expr(hier_name_expr());
   vhdl_pcall_stmt *pc2 = new vhdl_pcall_stmt("sv_display_line");
   pc2->add_expr(line2);
   container->add_stmt(pc2);

   container->add_stmt(new vhdl_report_stmt(new vhdl_const_string(word),
                                            level));
   return 0;
}

// $swrite(dest, fmt, args...): format into a string and store it in
// dest as packed 8-bit ASCII, right-justified and zero-filled (the
// Verilog string-in-reg convention, what %s/%0s of the reg expects).
// The formatter is the $display machinery starting at parameter 1.
// $sformat(dest, fmt, args...) is the same with a format string always in
// parameter 1 (it was not translated: dest kept its old value). Only a
// literal format translates (the $display machinery parses it at translation
// time); a variable one is a located error.
static int draw_stask_swrite(vhdl_procedural *proc,
                             stmt_container *container,
                             ivl_statement_t stmt)
{
   const char *task = ivl_stmt_name(stmt);
   ivl_expr_t dst = ivl_stmt_parm(stmt, 0);
   if (dst == NULL || ivl_expr_type(dst) != IVL_EX_SIGNAL) {
      error("%s:%d: %s destination must be a simple register",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt), task);
      return 1;
   }
   if (strcmp(task, "$sformat") == 0) {
      ivl_expr_t fmt = ivl_stmt_parm_count(stmt) > 1
         ? ivl_stmt_parm(stmt, 1) : NULL;
      if (fmt == NULL || ivl_expr_type(fmt) != IVL_EX_STRING) {
         error("%s:%d: no VHDL translation for a $sformat format that is not "
               "a string literal", ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
         return 1;
      }
   }

   vhdl_expr *text = build_display_text(proc, container, stmt, 1);
   if (NULL == text)
      return 1;

   ivl_signal_t sig = ivl_expr_signal(dst);
   vhdl_var_ref *lhs =
      nexus_to_var_ref(proc->get_scope(), ivl_signal_nex(sig, 0));

   vhdl_fcall *conv = new vhdl_fcall("sv_str2vec", lhs->get_type());
   conv->add_expr(text);
   conv->add_expr(new vhdl_const_int(ivl_signal_width(sig)));

   // Same blocking-emulation shape as make_assignment: a signal target is
   // deposited (deposits_signal), or else registers as blocking so later
   // same-step reads insert wait-for-0
   vhdl_decl *decl = proc->get_scope()->get_decl(lhs->get_name());
   if (decl != NULL
       && decl->assignment_type() == vhdl_decl::ASSIGN_NONBLOCK
       && !deposits_signal(proc, lhs->get_name(), true)) {
      if (proc->get_scope()->allow_signal_assignment())
         proc->add_blocking_target(lhs);
      container->add_stmt(new vhdl_nbassign_stmt(lhs, conv));
   }
   else {
      if (decl != NULL
          && decl->assignment_type() == vhdl_decl::ASSIGN_NONBLOCK)
         proc->mark_deposited(lhs->get_name());
      container->add_stmt(new vhdl_assign_stmt(lhs, conv));
   }
   return 0;
}

/*
 * The signals a $monitor / $fmonitor companion process watches: every
 * architecture signal or port its text reads, added to mon's sensitivity
 * (skip: a name already there) and returned.  A word, bit or part select
 * with constant bounds is watched by itself, as vvp watches just that
 * (sys_monitor_calltf: a memory word, a part select), so a write to another
 * word of a memory prints nothing; any other read watches the whole signal.
 */
std::vector<std::string> monitor_watch(vhdl_process *mon, vhdl_scope *ascope,
                                       vhdl_expr *text, const std::string &skip)
{
   vhdl_var_set_t rd;
   text->find_vars(rd);
   set<string> seen;
   seen.insert(skip);
   std::vector<std::string> out;
   for (vhdl_var_set_t::const_iterator it = rd.begin(); it != rd.end(); ++it) {
      const string &nm = (*it)->get_name();
      // Only architecture-visible signals (a signal, a port, an alias of
      // one) can be in a sensitivity list.
      vhdl_decl *d = ascope->get_decl(nm);
      if (d == NULL || dynamic_cast<vhdl_var_decl*>(d) != NULL
          || dynamic_cast<vhdl_type_decl*>(d) != NULL
          || dynamic_cast<vhdl_component_decl*>(d) != NULL
          || dynamic_cast<vhdl_param_decl*>(d) != NULL)
         continue;
      string name = nm;
      // (a bit or part of a constant memory word too: `array[0][1]' is
      // watched as array(0)(1), as vvp watches that bit; a write to another
      // bit of the word printed a line again, ivtest pr2785294)
      if ((*it)->get_slice() != NULL
          && dynamic_cast<vhdl_const_int*>((*it)->get_slice()) != NULL
          && (*it)->extra_slices_constant()) {
         ostringstream ss;
         (*it)->emit(ss, 0);
         name = ss.str();
      }
      if (seen.insert(name).second) {
         mon->add_sensitivity(name);
         out.push_back(name);
      }
   }
   return out;
}

// $monitor / $strobe: both print at the END of a time step, reading settled
// values -- exactly a POSTPONED process. Each statement gets a companion
// postponed process holding its (re-evaluated) display text:
//
//  * $monitor arms its companion via an architecture-level integer signal
//    (sv_monitor_arm <= K); the companion is sensitive to the arm and to
//    every signal the text reads, and prints when armed -- once per settled
//    step, starting immediately on arming. A later $monitor re-arms a
//    different id, disarming this one (Verilog: one active monitor).
//    $monitoroff arms id 0; $monitoron restores the last armed id.
//  * $strobe bumps a per-statement request counter; the companion prints
//    (request - serviced) times at the end of that step.
static int g_monitor_count = 0;

static int draw_stask_monitor(vhdl_procedural *proc,
                              stmt_container *container,
                              ivl_statement_t stmt, bool is_monitor)
{
   vhdl_entity *ent = get_active_entity();
   if (NULL == ent) {
      error("$monitor/$strobe outside an entity context at %s:%d",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
      return 1;
   }
   vhdl_arch *arch = ent->get_arch();
   vhdl_scope *ascope = arch->get_scope();
   const int id = ++g_monitor_count;

   ostringstream pname;
   pname << (is_monitor ? "sv_monitor_" : "sv_strobe_") << id;
   vhdl_process *mon = new vhdl_process(pname.str().c_str());
   mon->set_postponed();

   vhdl_expr *text = build_display_text(NULL, mon->get_container(), stmt);
   if (NULL == text)
      return 1;

   if (is_monitor) {
      if (!ascope->have_declared("sv_monitor_arm")) {
         vhdl_signal_decl *d =
            new vhdl_signal_decl("sv_monitor_arm", vhdl_type::integer());
         d->set_initial(new vhdl_const_int(0));
         ascope->add_decl(d);
         vhdl_signal_decl *l =
            new vhdl_signal_decl("sv_monitor_last", vhdl_type::integer());
         l->set_initial(new vhdl_const_int(0));
         ascope->add_decl(l);
      }
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref("sv_monitor_arm", vhdl_type::integer()),
         new vhdl_const_int(id)));
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref("sv_monitor_last", vhdl_type::integer()),
         new vhdl_const_int(id)));

      vhdl_binop_expr *armed = new vhdl_binop_expr(
         new vhdl_var_ref("sv_monitor_arm", vhdl_type::integer()),
         VHDL_BINOP_EQ, new vhdl_const_int(id), vhdl_type::boolean());
      vhdl_if_stmt *iff = new vhdl_if_stmt(armed);
      iff->get_then_container()->add_stmt(new vhdl_report_stmt(text));
      mon->get_container()->add_stmt(iff);
      mon->add_sensitivity("sv_monitor_arm");
   }
   else {
      ostringstream req;
      req << "sv_strobe_req_" << id;
      vhdl_signal_decl *d =
         new vhdl_signal_decl(req.str(), vhdl_type::integer());
      d->set_initial(new vhdl_const_int(0));
      ascope->add_decl(d);
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref(req.str().c_str(), vhdl_type::integer()),
         new vhdl_binop_expr(
            new vhdl_var_ref(req.str().c_str(), vhdl_type::integer()),
            VHDL_BINOP_ADD, new vhdl_const_int(1), vhdl_type::integer())));

      // variable serviced : integer := 0;  print (req - serviced) times
      vhdl_var_decl *sv =
         new vhdl_var_decl("serviced", vhdl_type::integer());
      sv->set_initial(new vhdl_const_int(0));
      mon->get_scope()->add_decl(sv);

      vhdl_for_stmt *loop = new vhdl_for_stmt("P", new vhdl_const_int(1),
         new vhdl_binop_expr(
            new vhdl_var_ref(req.str().c_str(), vhdl_type::integer()),
            VHDL_BINOP_SUB,
            new vhdl_var_ref("serviced", vhdl_type::integer()),
            vhdl_type::integer()));
      loop->get_container()->add_stmt(new vhdl_report_stmt(text));
      mon->get_container()->add_stmt(loop);
      mon->get_container()->add_stmt(new vhdl_assign_stmt(
         new vhdl_var_ref("serviced", vhdl_type::integer()),
         new vhdl_var_ref(req.str().c_str(), vhdl_type::integer())));
      mon->add_sensitivity(req.str());
   }

   // Sensitivity: every signal the printed text reads (monitor re-prints on
   // any operand change), a constant word or part select by itself.
   if (is_monitor)
      monitor_watch(mon, ascope, text, "sv_monitor_arm");

   arch->add_stmt(mon);
   return 0;
}

/*
 * `$set_val(arr, idx0, idx1, ..., idxN, val)': sv-normalize's rewrite of a
 * one-line blocking `arr[idx0][idx1]...[idxN] = val;' (iverilog once
 * rejected a chained select there). `arr' is a vector, whose packed
 * dimensions the indices select in, or an unpacked array (a memory): idx0
 * selects the word and the others select in the word's packed dimensions
 * (Hazard3's `req_stratified[i][j] = ...'; a memory was refused, and the
 * process lost the rest of its body without an error). As in Verilog, an
 * index counts from its dimension's declared bounds ([8:1], [0:7], a memory
 * [4:7]) and a store with an index outside its dimension is dropped:
 *
 *    SetVal_Idx_<n> := <idx0 from 0>;  ...      -- an index read once
 *    if SetVal_Idx_<n> >= 0 and SetVal_Idx_<n> <= <size-1> and ... then
 *       arr(<word>)(<bit offset> [+ w-1 downto <bit offset>]) <= val;
 *    end if;
 *
 * (constant indices are checked here instead). The select takes the
 * value's low bits (a wider select extends it), and the store is a blocking
 * assignment as make_assignment draws one.
 */
static vhdl_expr *variable_value(vhdl_expr *rhs, ivl_expr_t src);
static vhdl_abstract_assign_stmt *
assign_for(vhdl_decl::assign_type_t atype, vhdl_var_ref *lhs, vhdl_expr *rhs);
bool check_valid_assignment(vhdl_decl::assign_type_t atype,
                            vhdl_procedural *proc, ivl_statement_t stmt);

static int draw_stask_set_val(vhdl_procedural *proc,
                               stmt_container *container,
                               ivl_statement_t stmt)
{
   const char *file = ivl_stmt_file(stmt);
   const unsigned line = ivl_stmt_lineno(stmt);
   const int count = ivl_stmt_parm_count(stmt);
   if (count < 3) {
      error("%s:%d: $set_val takes a variable, one index or more and a value",
            file, line);
      return 1;
   }

   ivl_expr_t arr_expr = ivl_stmt_parm(stmt, 0);
   const bool memory = arr_expr != NULL
      && ivl_expr_type(arr_expr) == IVL_EX_ARRAY;
   if (arr_expr == NULL
       || (!memory && (ivl_expr_type(arr_expr) != IVL_EX_SIGNAL
                       || ivl_expr_oper1(arr_expr) != NULL
                       || ivl_signal_dimensions(ivl_expr_signal(arr_expr)) != 0))) {
      error("%s:%d: the first argument of $set_val must be a vector or a "
            "memory", file, line);
      return 1;
   }
   ivl_signal_t sig = ivl_expr_signal(arr_expr);
   ensure_signal_declared(sig);   // package/$unit-scope orphans
   const string signame(get_renamed_signal(sig));
   vhdl_decl *decl = proc->get_scope()->get_decl(signame);
   if (decl == NULL || decl->get_type() == NULL) {
      error("%s:%d: $set_val of %s, which has no VHDL declaration visible "
            "here", file, line, ivl_signal_name(sig));
      return 1;
   }
   if (ivl_signal_data_type(sig) == IVL_VT_REAL
       || (memory && (ivl_signal_dimensions(sig) != 1
                      || decl->get_type()->get_name() != VHDL_TYPE_ARRAY))) {
      error("%s:%d: no VHDL translation for $set_val of %s (a real, or a "
            "memory of more than one dimension)", file, line,
            ivl_signal_name(sig));
      return 1;
   }

   // The packed dimensions the indices after the word index select in, from
   // the outermost (dimension 0); a scalar is one dimension [0:0]
   std::vector<int> left, right, size;
   const unsigned pd = ivl_signal_packed_dimensions(sig);
   for (unsigned d = 0; d < pd; d++) {
      left.push_back(ivl_signal_packed_msb(sig, d));
      right.push_back(ivl_signal_packed_lsb(sig, d));
   }
   if (pd == 0) {
      left.push_back(0);
      right.push_back(0);
   }
   for (size_t d = 0; d < left.size(); d++)
      size.push_back((left[d] >= right[d] ? left[d] - right[d]
                                          : right[d] - left[d]) + 1);
   const int first = memory ? 2 : 1;            // the first packed index
   const int nbit = count - 1 - first;          // packed indices
   if (nbit < 0 || nbit > (int)size.size()) {
      error("%s:%d: $set_val of %s has %d indices for %u dimensions", file,
            line, ivl_signal_name(sig), count - 2,
            (unsigned)size.size() + (memory ? 1 : 0));
      return 1;
   }
   int inner_w = 1;                              // the select's width
   for (size_t d = nbit; d < size.size(); d++)
      inner_w *= size[d];

   // Each index from 0 at its dimension's least significant end (a word
   // index from the array's lowest address, the canonical word), with the
   // range it must lie in. A constant index is folded here.
   struct index_t {
      vhdl_expr *expr;     // NULL: constant `value'
      long value;
      int hi;              // in range: 0 .. hi
   };
   std::vector<index_t> idx;
   vhdl_type integer(VHDL_TYPE_INTEGER);
   for (int p = 1; p < count - 1; p++) {
      ivl_expr_t ie = ivl_stmt_parm(stmt, p);
      const bool word = memory && p == 1;
      const int d = p - first;                   // packed dimension
      long bias;           // canonical = sign * (index - bias)
      int sign = 1, hi;
      if (word) {
         bias = ivl_signal_array_base(sig);
         hi = (int)ivl_signal_array_count(sig) - 1;
      }
      else {
         bias = right[d];
         if (left[d] < right[d])
            sign = -1;
         hi = size[d] - 1;
      }
      index_t ix = { NULL, 0, hi };
      if (ivl_expr_type(ie) == IVL_EX_NUMBER && number_is_long(ie)) {
         ix.value = sign * (get_number_as_long(ie) - bias);
      }
      else {
         vhdl_expr *v = translate_expr(ie);
         if (v == NULL)
            return 1;
         emit_wait_for_0(proc, container, stmt, v);
         v = index_to_integer(ie, v);
         // (no negative literal: VHDL has no `x - -1')
         if (sign > 0 && bias > 0)
            v = new vhdl_binop_expr(v, VHDL_BINOP_SUB,
                                    new vhdl_const_int(bias),
                                    vhdl_type::integer());
         else if (sign > 0 && bias < 0)
            v = new vhdl_binop_expr(v, VHDL_BINOP_ADD,
                                    new vhdl_const_int(-bias),
                                    vhdl_type::integer());
         else if (sign < 0 && bias >= 0)
            v = new vhdl_binop_expr(new vhdl_const_int(bias), VHDL_BINOP_SUB,
                                    v, vhdl_type::integer());
         else if (sign < 0)
            v = new vhdl_binop_expr(
               new vhdl_binop_expr(new vhdl_const_int(0), VHDL_BINOP_SUB, v,
                                   vhdl_type::integer()),
               VHDL_BINOP_SUB, new vhdl_const_int(-bias), vhdl_type::integer());
         ix.expr = v;
      }
      idx.push_back(ix);
   }

   ivl_expr_t val_e = ivl_stmt_parm(stmt, count - 1);
   vhdl_expr *val = translate_expr(val_e);
   if (val == NULL)
      return 1;
   emit_wait_for_0(proc, container, stmt, val);

   // A constant index outside its dimension: Verilog drops the store
   for (size_t k = 0; k < idx.size(); k++)
      if (idx[k].expr == NULL && (idx[k].value < 0 || idx[k].value > idx[k].hi))
         return 0;

   // The other indices are read once, into integers, and guard the store
   static int set_val_count = 0;
   vhdl_binop_expr *guard = NULL;
   std::vector<vhdl_expr*> at;                   // each index, as used
   for (size_t k = 0; k < idx.size(); k++) {
      if (idx[k].expr == NULL) {
         at.push_back(new vhdl_const_int(idx[k].value));
         continue;
      }
      ostringstream nm;
      nm << "SetVal_Idx_" << set_val_count++;
      proc->get_scope()->add_decl(new vhdl_var_decl(nm.str(),
                                                    vhdl_type::integer()));
      vhdl_assign_stmt *cap = new vhdl_assign_stmt(
         new vhdl_var_ref(nm.str(), vhdl_type::integer()), idx[k].expr);
      if (k == 0) {
         ostringstream cs;
         cs << "$set_val (" << file << ":" << line << "): a store outside "
            << ivl_signal_basename(sig) << " is dropped";
         cap->set_comment(cs.str());
      }
      container->add_stmt(cap);
      if (guard == NULL)
         guard = new vhdl_binop_expr(VHDL_BINOP_AND, vhdl_type::boolean());
      guard->add_expr(new vhdl_binop_expr(
         new vhdl_var_ref(nm.str(), vhdl_type::integer()), VHDL_BINOP_GEQ,
         new vhdl_const_int(0), vhdl_type::boolean()));
      guard->add_expr(new vhdl_binop_expr(
         new vhdl_var_ref(nm.str(), vhdl_type::integer()), VHDL_BINOP_LEQ,
         new vhdl_const_int(idx[k].hi), vhdl_type::boolean()));
      at.push_back(new vhdl_var_ref(nm.str(), vhdl_type::integer()));
   }

   // The bit offset of the select in the vector or word: the packed
   // indices weighted by the sizes of the dimensions inside theirs
   vhdl_expr *flat = NULL;
   for (int p = 0; p < nbit; p++) {
      int weight = 1;
      for (size_t d = p + 1; d < size.size(); d++)
         weight *= size[d];
      vhdl_expr *term = at[(memory ? 1 : 0) + p];
      if (weight != 1)
         term = new vhdl_binop_expr(term, VHDL_BINOP_MULT,
                                    new vhdl_const_int(weight),
                                    vhdl_type::integer());
      flat = flat == NULL ? term
         : new vhdl_binop_expr(flat, VHDL_BINOP_ADD, term, vhdl_type::integer());
   }

   vhdl_var_ref *lhs = new vhdl_var_ref(signame,
                                        new vhdl_type(*decl->get_type()));
   if (memory)
      lhs->set_slice(at[0], 0);                 // the word
   const vhdl_type *t = lhs->get_type();         // a vector, or a scalar
   if (flat != NULL) {
      if (t->get_name() == VHDL_TYPE_LOGIC3D_VECTOR
          || t->get_name() == VHDL_TYPE_UNSIGNED
          || t->get_name() == VHDL_TYPE_SIGNED) {
         if (memory)
            lhs->slice_element(flat, inner_w - 1);
         else
            lhs->set_slice(flat, inner_w - 1);
      }
      else
         delete flat;   // a scalar's only bit (the guard checked the index)
   }

   // The value at the select's width: its low bits, or extended
   const vhdl_type *lt = lhs->get_type();
   if (lt->get_name() == VHDL_TYPE_LOGIC3D && val->get_type() != NULL
       && val->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      // One bit of a vector value, by way of a variable of its type: bit 0
      // keeps an x or z (a cast would read its value plane)
      ostringstream vn;
      vn << "SetVal_Val_" << set_val_count++;
      const int vw = val->get_type()->get_width();
      proc->get_scope()->add_decl(new vhdl_var_decl(
         vn.str(), vhdl_type::logic3d_vector(vw - 1, 0)));
      container->add_stmt(new vhdl_assign_stmt(
         new vhdl_var_ref(vn.str(), vhdl_type::logic3d_vector(vw - 1, 0)),
         val));
      vhdl_var_ref *b0 = new vhdl_var_ref(vn.str(),
                                          vhdl_type::logic3d_vector(vw - 1, 0));
      b0->set_slice(new vhdl_const_int(0), 0);
      val = b0;
   }
   else
      val = val->cast(lt);
   val = variable_value(val, val_e);

   // A blocking assignment, as make_assignment draws one: a deposit where
   // deposits_signal says so, else a signal assignment that the process's
   // later reads see through its blocking-target machinery
   vhdl_decl::assign_type_t atype = decl->assignment_type();
   if (atype == vhdl_decl::ASSIGN_NONBLOCK
       && deposits_signal(proc, lhs->get_name(), true)) {
      atype = vhdl_decl::ASSIGN_BLOCK;
      proc->mark_deposited(lhs->get_name());
   }
   if (!check_valid_assignment(atype, proc, stmt))
      return 1;
   if (atype == vhdl_decl::ASSIGN_NONBLOCK)
      proc->add_blocking_target(lhs);

   stmt_container *where = container;
   if (guard != NULL) {
      vhdl_if_stmt *in_range = new vhdl_if_stmt(guard);
      container->add_stmt(in_range);
      where = in_range->get_then_container();
   }
   where->add_stmt(assign_for(atype, lhs, val));
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

// fileio.cc: the arguments of the file tasks, as vvp reads them
vhdl_expr *fileio_int32(vhdl_procedural *proc, stmt_container *container,
                        ivl_statement_t stmt, ivl_expr_t e);
vhdl_expr *fileio_string(vhdl_procedural *proc, stmt_container *container,
                         ivl_statement_t stmt, ivl_expr_t e);
const char *fileio_vpi_kind(ivl_expr_t e);
bool fileio_real_arg(ivl_expr_t e);
int fileio_unsupported(stmt_container *container, ivl_statement_t stmt,
                       const char *why);
std::string fileio_loc(ivl_statement_t stmt);

// emit_wait_for_0 for the file tasks (fileio.cc): the wait-for-0 a read of a
// blocking target needs first
void fileio_wait_for_0(vhdl_procedural *proc, stmt_container *container,
                       ivl_statement_t stmt, vhdl_expr *expr)
{
   emit_wait_for_0(proc, container, stmt, expr);
}

/*
 * $readmemh / $readmemb / $writememh / $writememb (file, mem [, start
 * [, finish]]), on the runtime of nvc's logic3d_types_pkg, with vvp's rules
 * and messages (vpi/sys_readmem.c):
 *
 *   sv_readmem_load(<file>, <hex>, "$readmemh", "<file>:<line>",
 *                   "<scope>.<mem>", <left>, <right>, <W>, "<vpi kind>",
 *                   <has start>, <start>, <real>, <has finish>, <finish>, <real>);
 *   sv_write_buf(sv_memfile_msgs);
 *   for sv_rm_i in 0 to sv_readmem_count - 1 loop
 *     mem(sv_readmem_addr(sv_rm_i) - <lowest address>) := sv_readmem_word(sv_rm_i, W);
 *   end loop;
 *
 *   sv_writemem_open(<file>, <hex>, "$writememh", <the rest, less W>);
 *   sv_write_buf(sv_memfile_msgs);
 *   for sv_wm_i in 0 to sv_writemem_count - 1 loop
 *     sv_writemem_word(sv_hstr(to_std_logic_vector(
 *        mem(sv_writemem_addr(sv_wm_i) - <lowest address>))));
 *   end loop;
 *   sv_writemem_close;
 *
 * The runtime checks the arguments and reads the file, and yields only the
 * words that go into the memory, with their Verilog addresses; its messages
 * print through the $display line buffer.  The VHDL array is (count-1 downto
 * 0), 0 the word at the lowest Verilog address (declare_one_signal).  A word
 * is written the way make_assignment writes a blocking target: a signal is
 * deposited (:=) in an initializing process and, once deposited there, ever
 * after; else it is assigned (<=) and registered as a blocking target, so a
 * read that follows in the process waits for it.  A 1-bit word takes
 * sv_readmem_bit.  A memory this has no translation for -- one in another
 * module, a real one -- gets the located "Unsupported system task" comment
 * every untranslated task gets (vamos reports it), never a silent drop.
 */
static int draw_stask_readmem(vhdl_procedural *proc, stmt_container *container,
                              ivl_statement_t stmt, bool hex, bool writing)
{
   const char *name = ivl_stmt_name(stmt);
   const unsigned nparms = ivl_stmt_parm_count(stmt);
   ivl_expr_t fe = nparms > 0 ? ivl_stmt_parm(stmt, 0) : NULL;
   ivl_expr_t me = nparms > 1 ? ivl_stmt_parm(stmt, 1) : NULL;
   ivl_expr_t se = nparms > 2 ? ivl_stmt_parm(stmt, 2) : NULL;
   ivl_expr_t ee = nparms > 3 ? ivl_stmt_parm(stmt, 3) : NULL;
   // vvp's sys_mem_compiletf: these stop the compile (an ivtest CE)
   if (fe == NULL || me == NULL) {
      error("%s:%d: %s requires two arguments (a file name and a memory)",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt), name);
      return 1;
   }
   // A whole memory passed to a system task is an IVL_EX_ARRAY expression
   if (!(ivl_expr_type(me) == IVL_EX_ARRAY
         || (ivl_expr_type(me) == IVL_EX_SIGNAL && ivl_expr_oper1(me) == NULL))
       || ivl_signal_dimensions(ivl_expr_signal(me)) == 0) {
      error("%s:%d: %s's second argument must be a memory.", ivl_stmt_file(stmt),
            ivl_stmt_lineno(stmt), name);
      return 1;
   }
   if (ivl_signal_dimensions(ivl_expr_signal(me)) != 1)
      return fileio_unsupported(container, stmt, "a memory of more than one "
                                "dimension");
   ivl_signal_t sig = ivl_expr_signal(me);
   if (ivl_signal_data_type(sig) == IVL_VT_REAL)
      return fileio_unsupported(container, stmt, "a memory of reals has no "
                                "word format");
   const string mname(get_renamed_signal(sig));
   vhdl_decl *decl = proc->get_scope()->get_decl(mname);
   if (decl == NULL || decl->get_type() == NULL
       || decl->get_type()->get_name() != VHDL_TYPE_ARRAY
       || decl->get_type()->get_base() == NULL)
      return fileio_unsupported(container, stmt, "the memory is not visible "
                                "here (it is in another module)");
   const vhdl_type *etype = decl->get_type()->get_base();
   const int width = ivl_signal_width(sig);
   if (!((width == 1 && etype->get_name() == VHDL_TYPE_LOGIC3D)
         || (width > 1 && etype->get_name() == VHDL_TYPE_LOGIC3D_VECTOR
             && etype->get_width() == width)))
      return fileio_unsupported(container, stmt, "the memory's words are not "
                                "logic vectors");

   // The declared range [left:right]; t-dll.cc: the base is the lower
   // address, "swapped" a [high:low] declaration
   const int lo = ivl_signal_array_base(sig);
   const int cnt = (int)ivl_signal_array_count(sig);
   const bool swapped = ivl_signal_array_addr_swapped(sig) != 0;
   const int left = swapped ? lo + cnt - 1 : lo;
   const int right = swapped ? lo : lo + cnt - 1;
   const string full = string(ivl_scope_name(ivl_signal_scope(sig))) + "."
      + ivl_signal_basename(sig);

   vhdl_expr *fname = fileio_string(proc, container, stmt, fe);
   if (fname == NULL)
      return 1;
   vhdl_expr *start = NULL, *finish = NULL;
   if (se != NULL && (start = fileio_int32(proc, container, stmt, se)) == NULL)
      return 1;
   if (ee != NULL && (finish = fileio_int32(proc, container, stmt, ee)) == NULL)
      return 1;

   vhdl_pcall_stmt *begin = new vhdl_pcall_stmt(writing ? "sv_writemem_open"
                                                        : "sv_readmem_load");
   begin->add_expr(fname);
   begin->add_expr(new vhdl_var_ref(hex ? "true" : "false", vhdl_type::boolean()));
   begin->add_expr(new vhdl_const_string(name));
   begin->add_expr(new vhdl_const_string(fileio_loc(stmt)));
   begin->add_expr(new vhdl_const_string(full));
   begin->add_expr(new vhdl_const_int(left));
   begin->add_expr(new vhdl_const_int(right));
   if (!writing)
      begin->add_expr(new vhdl_const_int(width));
   begin->add_expr(new vhdl_const_string(fileio_vpi_kind(fe)));
   begin->add_expr(new vhdl_var_ref(start ? "true" : "false",
                                    vhdl_type::boolean()));
   begin->add_expr(start ? start : new vhdl_const_int(0));
   begin->add_expr(new vhdl_var_ref(fileio_real_arg(se) ? "true" : "false",
                                    vhdl_type::boolean()));
   begin->add_expr(new vhdl_var_ref(finish ? "true" : "false",
                                    vhdl_type::boolean()));
   begin->add_expr(finish ? finish : new vhdl_const_int(0));
   begin->add_expr(new vhdl_var_ref(fileio_real_arg(ee) ? "true" : "false",
                                    vhdl_type::boolean()));

   // The word's VHDL index: its Verilog address less the lowest one
   const char *iv = writing ? "sv_wm_i" : "sv_rm_i";
   vhdl_fcall *addr = new vhdl_fcall(writing ? "sv_writemem_addr"
                                             : "sv_readmem_addr",
                                     vhdl_type::integer());
   addr->add_expr(new vhdl_var_ref(iv, vhdl_type::integer()));
   vhdl_expr *index = addr;
   if (lo != 0)
      index = new vhdl_binop_expr(addr, lo > 0 ? VHDL_BINOP_SUB : VHDL_BINOP_ADD,
                                  new vhdl_const_int(lo > 0 ? lo : -lo),
                                  vhdl_type::integer());
   vhdl_var_ref *word = new vhdl_var_ref(mname.c_str(),
                                         new vhdl_type(*decl->get_type()));
   word->set_slice(index);

   // $writemem reads the memory: a blocking write to it earlier in this
   // process lands first (the wait-for-0 of any read)
   if (writing)
      fileio_wait_for_0(proc, container, stmt, word);
   container->add_stmt(begin);
   vhdl_pcall_stmt *msgs = new vhdl_pcall_stmt("sv_write_buf");
   msgs->add_expr(new vhdl_fcall("sv_memfile_msgs", vhdl_type::string()));
   container->add_stmt(msgs);

   vhdl_fcall *count = new vhdl_fcall(writing ? "sv_writemem_count"
                                              : "sv_readmem_count",
                                      vhdl_type::integer());
   vhdl_for_stmt *loop = new vhdl_for_stmt(
      iv, new vhdl_const_int(0),
      new vhdl_binop_expr(count, VHDL_BINOP_SUB, new vhdl_const_int(1),
                          vhdl_type::integer()));

   if (writing) {
      vhdl_fcall *slv = new vhdl_fcall("to_std_logic_vector",
                                       vhdl_type::std_logic_vector(width - 1, 0));
      slv->add_expr(word);
      vhdl_fcall *txt = new vhdl_fcall(hex ? "sv_hstr" : "sv_bstr",
                                       vhdl_type::string());
      txt->add_expr(slv);
      vhdl_pcall_stmt *put = new vhdl_pcall_stmt("sv_writemem_word");
      put->add_expr(txt);
      loop->get_container()->add_stmt(put);
      container->add_stmt(loop);
      container->add_stmt(new vhdl_pcall_stmt("sv_writemem_close"));
      return 0;
   }

   vhdl_expr *value;
   if (width == 1) {
      vhdl_fcall *f = new vhdl_fcall("sv_readmem_bit", vhdl_type::logic3d());
      f->add_expr(new vhdl_var_ref(iv, vhdl_type::integer()));
      value = f;
   }
   else {
      vhdl_fcall *f = new vhdl_fcall("sv_readmem_word",
                                     vhdl_type::logic3d_vector(width - 1, 0));
      f->add_expr(new vhdl_var_ref(iv, vhdl_type::integer()));
      f->add_expr(new vhdl_const_int(width));
      value = f;
   }

   // A blocking write, make_assignment's way: deposited (:=) where
   // deposits_signal says so, and then read back at once; else assigned (<=)
   // and registered as a blocking target, which a later read waits for
   vhdl_decl::assign_type_t atype = decl->assignment_type();
   if (atype == vhdl_decl::ASSIGN_NONBLOCK) {
      if (deposits_signal(proc, mname, true)) {
         atype = vhdl_decl::ASSIGN_BLOCK;
         proc->mark_deposited(mname);
      }
      else {
         if (!proc->get_scope()->allow_signal_assignment()) {
            error("%s:%d: %s writes memory %s, a signal, where VHDL cannot "
                  "assign one (a function)", ivl_stmt_file(stmt),
                  ivl_stmt_lineno(stmt), name, ivl_signal_basename(sig));
            return 1;
         }
         proc->add_blocking_target(word);
      }
   }
   if (atype == vhdl_decl::ASSIGN_NONBLOCK)
      loop->get_container()->add_stmt(new vhdl_nbassign_stmt(word, value));
   else
      loop->get_container()->add_stmt(new vhdl_assign_stmt(word, value));
   container->add_stmt(loop);
   return 0;
}

// fileio.cc: $fdisplay, $fwrite, $fstrobe, $fmonitor (and their b/h/o
// forms), $fclose, $fflush
bool fileio_task(const char *name);
int draw_stask_fileio(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt);

/*
 * Generate VHDL for system tasks (like $display). Not all of
 * these are supported.
 */
static int draw_stask(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt)
{
   const char *name = ivl_stmt_name(stmt);

   // `$random(seed);' -- a system function called as a task: its value is
   // dropped, its draw (a seed's advance) stands
   if (get_sv2vhdl_mode()
       && (strcmp(name, "$random") == 0 || strcmp(name, "$urandom") == 0))
      return draw_stask_random(proc, container, stmt);

   if (strcmp(name, "$display") == 0)
      return draw_stask_display(proc, container, stmt, true);
   else if (strcmp(name, "$write") == 0)
      return draw_stask_display(proc, container, stmt, false);
   else if (strcmp(name, "$swrite") == 0
            || (get_sv2vhdl_mode() && strcmp(name, "$sformat") == 0))
      return draw_stask_swrite(proc, container, stmt);
   else if (strcmp(name, "$monitor") == 0)
      return draw_stask_monitor(proc, container, stmt, true);
   else if (strcmp(name, "$strobe") == 0)
      return draw_stask_monitor(proc, container, stmt, false);
   else if (strcmp(name, "$monitoroff") == 0) {
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref("sv_monitor_arm", vhdl_type::integer()),
         new vhdl_const_int(0)));
      return 0;
   }
   else if (strcmp(name, "$monitoron") == 0) {
      container->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref("sv_monitor_arm", vhdl_type::integer()),
         new vhdl_var_ref("sv_monitor_last", vhdl_type::integer())));
      return 0;
   }
   else if (strcmp(name, "$finish") == 0)
      return draw_stask_finish(proc, container, stmt);
   else if (get_sv2vhdl_mode() && strcmp(name, "$stop") == 0)
      return draw_stask_stop(proc, container, stmt);
   else if (get_sv2vhdl_mode()
            && (strcmp(name, "$fatal") == 0 || strcmp(name, "$error") == 0
                || strcmp(name, "$warning") == 0
                || strcmp(name, "$info") == 0))
      return draw_stask_severity(proc, container, stmt);
   else if (strcmp(name, "$set_val") == 0)
      return draw_stask_set_val(proc, container, stmt);
   else if (strcmp(name, "$readmemh") == 0)
      return draw_stask_readmem(proc, container, stmt, true, false);
   else if (strcmp(name, "$readmemb") == 0)
      return draw_stask_readmem(proc, container, stmt, false, false);
   else if (get_sv2vhdl_mode() && strcmp(name, "$writememh") == 0)
      return draw_stask_readmem(proc, container, stmt, true, true);
   else if (get_sv2vhdl_mode() && strcmp(name, "$writememb") == 0)
      return draw_stask_readmem(proc, container, stmt, false, true);
   else if (get_sv2vhdl_mode() && fileio_task(name))
      return draw_stask_fileio(proc, container, stmt);
   else if (strncmp(name, "$ivl_queue_method$", 18) == 0
            || strncmp(name, "$ivl_darray_method$", 19) == 0)
      return draw_queue_method(proc, container, stmt);
   else if (strcmp(name, "$timeformat") == 0) {
      // $timeformat(units, precision, suffix, min_width) -> set the global %t
      // format via sv_set_timeformat. Args are constant in practice.
      vhdl_pcall_stmt *pc = new vhdl_pcall_stmt("sv_set_timeformat");
      vhdl_type itype(VHDL_TYPE_INTEGER);
      for (int a = 0; a < 4; a++) {
         ivl_expr_t pe = ivl_stmt_parm(stmt, a);
         vhdl_expr *ve = pe ? translate_expr(pe) : NULL;
         if (ve == NULL) {   // fall back to a harmless default
            pc->add_expr(a == 2 ? (vhdl_expr*)new vhdl_const_string("")
                                : (vhdl_expr*)new vhdl_const_int(0));
         } else if (a == 2) {
            // The suffix must be a VHDL string: translate_expr now packs
            // string literals into logic3d vectors (their Verilog VALUE
            // form), so take the text directly. ivl_expr_string() writes a
            // NUL (the empty string "" is one), a quote, a backslash and any
            // other unprintable character as an octal escape: decode them as
            // the $display path does; vhdl_const_string drops the NULs.
            if (ivl_expr_type(pe) == IVL_EX_STRING) {
               string suffix;
               for (const char *s = ivl_expr_string(pe); *s; s++) {
                  if (*s == '\\') {
                     suffix += parse_octal(s + 1);
                     s += 3;
                  }
                  else
                     suffix += *s;
               }
               pc->add_expr(new vhdl_const_string(suffix));
            }
            else
               pc->add_expr(ve);
         } else {
            pc->add_expr(ve->cast(&itype));   // units/precision/min_width
         }
      }
      container->add_stmt(pc);
      return 0;
   }
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
 * disable (Verilog `disable <scope>', SV `return'). A disable leaves the
 * named block, task or function it names, wherever inside it it stands.
 * Such a scope's body is drawn inside
 *
 *    sv_dis_<n>: loop
 *       <body>
 *       exit sv_dis_<n>;
 *    end loop sv_dis_<n>;
 *
 * when a disable inside it (or inside a task it calls) names it, and the
 * disable is `exit sv_dis_<n>;'; a function's is `return <f>_Result;'. A
 * disable of a scope that does not enclose it in the same process (another
 * process's block, a task from outside it, `disable fork') has no
 * translation: a located error. (Every disable used to be drawn as `null',
 * so `return x' in a function and `disable blk' were silently ignored.)
 */
namespace {
struct disable_target_t {
   ivl_scope_t scope;
   std::string label;      // the loop to exit; empty: a function (return)
   std::string result;     // a function's result variable
};
}
static std::vector<disable_target_t> g_disable_targets;
static std::vector<std::vector<disable_target_t> > g_disable_saved;
static int g_disable_count = 0;

static void save_loop_targets();
static void restore_loop_targets();

void begin_function_disables(ivl_scope_t fscope, const std::string &result)
{
   g_disable_saved.push_back(g_disable_targets);
   g_disable_targets.clear();
   disable_target_t t;
   t.scope = fscope;
   t.result = result;
   g_disable_targets.push_back(t);
   save_loop_targets();      // and break/continue: the function's own loops
}

void end_function_disables()
{
   assert(!g_disable_saved.empty());
   g_disable_targets = g_disable_saved.back();
   g_disable_saved.pop_back();
   restore_loop_targets();
}

// Whether a disable of `target' stands anywhere in `s' (or in a task it calls)
static bool stmt_disables(ivl_statement_t s, ivl_scope_t target,
                          std::set<ivl_scope_t> &tasks_seen)
{
   if (s == NULL)
      return false;
   switch (ivl_statement_type(s)) {
   case IVL_ST_DISABLE:
      return ivl_stmt_call(s) == target;
   case IVL_ST_BLOCK:
   case IVL_ST_FORK:
   case IVL_ST_FORK_JOIN_ANY:
   case IVL_ST_FORK_JOIN_NONE:
      for (unsigned i = 0; i < ivl_stmt_block_count(s); i++)
         if (stmt_disables(ivl_stmt_block_stmt(s, i), target, tasks_seen))
            return true;
      return false;
   case IVL_ST_CONDIT:
      return stmt_disables(ivl_stmt_cond_true(s), target, tasks_seen)
         || stmt_disables(ivl_stmt_cond_false(s), target, tasks_seen);
   case IVL_ST_CASE:
   case IVL_ST_CASER:
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
      for (unsigned i = 0; i < ivl_stmt_case_count(s); i++)
         if (stmt_disables(ivl_stmt_case_stmt(s, i), target, tasks_seen))
            return true;
      return false;
   case IVL_ST_FORLOOP:
      return stmt_disables(ivl_stmt_init_stmt(s), target, tasks_seen)
         || stmt_disables(ivl_stmt_sub_stmt(s), target, tasks_seen)
         || stmt_disables(ivl_stmt_step_stmt(s), target, tasks_seen);
   case IVL_ST_DELAY:
   case IVL_ST_DELAYX:
   case IVL_ST_WAIT:
   case IVL_ST_WHILE:
   case IVL_ST_DO_WHILE:
   case IVL_ST_FOREVER:
   case IVL_ST_REPEAT:
      return stmt_disables(ivl_stmt_sub_stmt(s), target, tasks_seen);
   case IVL_ST_UTASK:
      {
         ivl_scope_t t = ivl_stmt_call(s);
         if (t == NULL || !tasks_seen.insert(t).second)
            return false;
         return stmt_disables(ivl_scope_def(t), target, tasks_seen);
      }
   default:
      return false;
   }
}

// When a disable inside `body' names `scope': a fresh `sv_dis_<n>: loop',
// with `scope' pushed as a disable target (pop with end_disable_scope).
static vhdl_labeled_loop_stmt *begin_disable_scope(ivl_scope_t scope,
                                                   ivl_statement_t body)
{
   std::set<ivl_scope_t> seen;
   if (scope == NULL || !stmt_disables(body, scope, seen))
      return NULL;
   std::ostringstream ss;
   ss << "sv_dis_" << ++g_disable_count;
   vhdl_labeled_loop_stmt *loop = new vhdl_labeled_loop_stmt(ss.str());
   ostringstream cs;
   cs << "Leaves " << ivl_scope_name(scope) << " on a disable";
   loop->set_comment(cs.str());
   disable_target_t t;
   t.scope = scope;
   t.label = ss.str();
   g_disable_targets.push_back(t);
   return loop;
}

// The loop gets its closing `exit <label>;' and joins the container
static void end_disable_scope(stmt_container *container,
                              vhdl_labeled_loop_stmt *loop)
{
   assert(!g_disable_targets.empty());
   g_disable_targets.pop_back();
   loop->get_container()->add_stmt(new vhdl_exit_stmt(loop->get_label()));
   container->add_stmt(loop);
}

static int draw_disable(vhdl_procedural *, stmt_container *container,
                        ivl_statement_t stmt)
{
   ivl_scope_t target = ivl_stmt_call(stmt);
   for (std::vector<disable_target_t>::reverse_iterator it =
           g_disable_targets.rbegin(); it != g_disable_targets.rend(); ++it) {
      if (it->scope != target)
         continue;
      vhdl_seq_stmt *s;
      if (it->label.empty())
         s = new vhdl_return_stmt(it->result);
      else
         s = new vhdl_exit_stmt(it->label);
      ostringstream cs;
      cs << (ivl_stmt_flow_control(stmt) ? "return" : "disable") << " "
         << ivl_scope_name(target) << " (" << ivl_stmt_file(stmt) << ":"
         << ivl_stmt_lineno(stmt) << ")";
      s->set_comment(cs.str());
      container->add_stmt(s);
      return 0;
   }
   if (target == NULL)
      error("unsupported construct (fork) at %s:%d: disable fork has no VHDL "
            "translation", ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
   else
      error("%s:%d: disable %s has no VHDL translation: only a block, task "
            "or function that encloses the disable statement, in the same "
            "process, can be disabled", ivl_stmt_file(stmt),
            ivl_stmt_lineno(stmt), ivl_scope_name(target));
   return 1;
}

/*
 * SystemVerilog break and continue (IVL_ST_BREAK, IVL_ST_CONTINUE) leave the
 * innermost loop, or the rest of its current pass. A loop whose body holds
 * one is drawn as
 *
 *    sv_brk_<n>: loop                 -- only with a break
 *       <the loop>                    -- while/for/repeat/forever/do-while
 *          sv_cont_<n>: loop          -- only with a continue
 *             <body>
 *             exit sv_cont_<n>;
 *          end loop sv_cont_<n>;
 *          <a for loop's step, the do-while test>
 *       ...
 *       exit sv_brk_<n>;
 *    end loop sv_brk_<n>;
 *
 * break is `exit sv_brk_<n>;', continue `exit sv_cont_<n>;' -- so a for
 * loop's step and a do-while's test still run after a continue, as in C,
 * and a labeled exit cannot be caught by a disable loop in between.
 */
namespace {
struct loop_target_t {
   std::string brk, cont;   // the labels; empty when the body has none
};
}
static std::vector<loop_target_t> g_loop_targets;
static std::vector<std::vector<loop_target_t> > g_loop_saved;
static int g_loop_jump_count = 0;

// A function body is drawn apart from its caller (translate_ufunc): its
// loops are its own.
static void save_loop_targets()
{
   g_loop_saved.push_back(g_loop_targets);
   g_loop_targets.clear();
}

static void restore_loop_targets()
{
   assert(!g_loop_saved.empty());
   g_loop_targets = g_loop_saved.back();
   g_loop_saved.pop_back();
}

// Whether a statement of type `kind' (IVL_ST_BREAK or IVL_ST_CONTINUE) in
// `s', a loop's body, belongs to that loop: not one inside a nested loop
// (or in a task it calls, which cannot leave the caller's loop)
static bool loop_body_jumps(ivl_statement_t s, ivl_statement_type_t kind)
{
   if (s == NULL)
      return false;
   switch (ivl_statement_type(s)) {
   case IVL_ST_BREAK:
   case IVL_ST_CONTINUE:
      return ivl_statement_type(s) == kind;
   case IVL_ST_BLOCK:
      for (unsigned i = 0; i < ivl_stmt_block_count(s); i++)
         if (loop_body_jumps(ivl_stmt_block_stmt(s, i), kind))
            return true;
      return false;
   case IVL_ST_CONDIT:
      return loop_body_jumps(ivl_stmt_cond_true(s), kind)
         || loop_body_jumps(ivl_stmt_cond_false(s), kind);
   case IVL_ST_CASE:
   case IVL_ST_CASER:
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
      for (unsigned i = 0; i < ivl_stmt_case_count(s); i++)
         if (loop_body_jumps(ivl_stmt_case_stmt(s, i), kind))
            return true;
      return false;
   case IVL_ST_DELAY:
   case IVL_ST_DELAYX:
   case IVL_ST_WAIT:
      return loop_body_jumps(ivl_stmt_sub_stmt(s), kind);
   default:
      return false;
   }
}

// Open the break/continue targets of a loop whose body is `body': returns
// the break wrapper (NULL without a break), and sets `cont' to the
// continue wrapper (NULL without a continue). Close with end_loop_jumps.
static vhdl_labeled_loop_stmt *begin_loop_jumps(ivl_statement_t body,
                                                vhdl_labeled_loop_stmt *&cont)
{
   loop_target_t t;
   vhdl_labeled_loop_stmt *brk = NULL;
   cont = NULL;
   const bool has_brk = loop_body_jumps(body, IVL_ST_BREAK);
   const bool has_cont = loop_body_jumps(body, IVL_ST_CONTINUE);
   if (has_brk || has_cont) {
      const int n = ++g_loop_jump_count;
      if (has_brk) {
         ostringstream ss;
         ss << "sv_brk_" << n;
         t.brk = ss.str();
         brk = new vhdl_labeled_loop_stmt(t.brk);
         brk->set_comment("A break leaves the loop in it");
      }
      if (has_cont) {
         ostringstream ss;
         ss << "sv_cont_" << n;
         t.cont = ss.str();
         cont = new vhdl_labeled_loop_stmt(t.cont);
         cont->set_comment("One pass of the loop body: a continue ends it");
      }
   }
   g_loop_targets.push_back(t);   // even an empty one: it hides outer loops
   return brk;
}

// Close the targets begin_loop_jumps opened
static void end_loop_jumps()
{
   assert(!g_loop_targets.empty());
   g_loop_targets.pop_back();
}

// The statement the loop's container receives: `loop' wrapped in its break
// loop when it has one
static vhdl_seq_stmt *wrap_break(vhdl_labeled_loop_stmt *brk, vhdl_seq_stmt *loop)
{
   if (brk == NULL)
      return loop;
   brk->get_container()->add_stmt(loop);
   brk->get_container()->add_stmt(new vhdl_exit_stmt(brk->get_label()));
   return brk;
}

// The container a loop's body is drawn into: the continue wrapper's (which
// close_continue then closes into `loop_c') or the loop's own
static stmt_container *continue_body(vhdl_labeled_loop_stmt *cont,
                                     stmt_container *loop_c)
{
   return cont ? cont->get_container() : loop_c;
}

static void close_continue(vhdl_labeled_loop_stmt *cont, stmt_container *loop_c)
{
   if (cont == NULL)
      return;
   cont->get_container()->add_stmt(new vhdl_exit_stmt(cont->get_label()));
   loop_c->add_stmt(cont);
}

static int draw_break_continue(vhdl_procedural *, stmt_container *container,
                               ivl_statement_t stmt)
{
   const bool is_break = ivl_statement_type(stmt) == IVL_ST_BREAK;
   const std::string label = g_loop_targets.empty() ? std::string()
      : (is_break ? g_loop_targets.back().brk : g_loop_targets.back().cont);
   if (label.empty()) {
      error("%s:%d: %s outside a loop has no VHDL translation",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt),
            is_break ? "break" : "continue");
      return 1;
   }
   vhdl_exit_stmt *s = new vhdl_exit_stmt(label);
   ostringstream cs;
   cs << (is_break ? "break" : "continue") << " (" << ivl_stmt_file(stmt)
      << ":" << ivl_stmt_lineno(stmt) << ")";
   s->set_comment(cs.str());
   container->add_stmt(s);
   return 0;
}

static void reset_automatic_vars(vhdl_procedural *proc, stmt_container *container,
                                 ivl_scope_t scope, bool task);

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
         if (!seen_signal_before(sig)) {
            remember_signal(sig, proc->get_scope());
            // A block local is a variable of its own, declared under its
            // VHDL-safe name (sv2v's `integer __i' becomes `sig_i'); every
            // reference uses the renamed name. When that name is already
            // visible here -- a module signal or port of the same name,
            // matched case-insensitively -- give it a fresh one, or its uses
            // would read and write that signal instead.
            std::string name = make_safe_name(sig);
            if (proc->get_scope()->have_declared(name)) {
               std::string fresh = name + "_blk";
               for (int k = 2; proc->get_scope()->have_declared(fresh); k++)
                  fresh = name + "_blk" + std::to_string(k);
               name = fresh;
            }
            rename_signal(sig, name);
         }

         std::string safe_name = get_renamed_signal(sig);
         if (!proc->get_scope()->have_declared(safe_name)) {
            proc->get_scope()->add_decl
               (new vhdl_var_decl(safe_name, vhdl_type_for_signal(sig)));
         }
      }
   }

   // A block with its own scope (a named block, or one with declarations) is
   // the scope %m and the Scope line of $error & co. name inside it
   // (top.blk, as vvp does).
   ivl_scope_t prev_scope = get_active_scope();
   if (block_scope)
      set_active_scope(block_scope);

   // A named block that a disable inside it leaves: its statements go in a
   // loop the disable exits (begin_disable_scope)
   vhdl_labeled_loop_stmt *dloop = begin_disable_scope(block_scope, stmt);
   stmt_container *body = dloop ? dloop->get_container() : container;

   // An automatic block (inside an automatic task, say) starts afresh each
   // time it is entered: its variables are x again
   if (block_scope && ivl_scope_is_auto(block_scope))
      reset_automatic_vars(proc, body, block_scope, false);

   int rc = 0;
   int count = ivl_stmt_block_count(stmt);
   for (int i = 0; i < count && rc == 0; i++) {
      ivl_statement_t stmt_i = ivl_stmt_block_stmt(stmt, i);
      if (draw_stmt(proc, body, stmt_i,
                    dloop == NULL && is_last && i == count - 1) != 0)
         rc = 1;
   }
   if (dloop)
      end_disable_scope(container, dloop);
   set_active_scope(prev_scope);
   return rc;
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

   ensure_signal_declared(sig);   // package/$unit-scope orphans

   // An lvalue can carry BOTH an array word index and a bit/part offset
   // (mem[i][hi:lo] = ...) -- they compose, in that order.
   vhdl_expr *word = NULL, *base = NULL;
   vhdl_type integer(VHDL_TYPE_INTEGER);
   ivl_expr_t e_idx = ivl_lval_idx(lval);
   ivl_expr_t e_part = ivl_lval_part_off(lval);
   if (e_idx) {
      if ((word = translate_expr(e_idx)) == NULL)
         return NULL;
      word = index_to_integer(e_idx, word);
   }
   if (e_part) {
      if ((base = translate_expr(e_part)) == NULL)
         return NULL;
      base = index_to_integer(e_part, base);
   }

   unsigned lval_width = ivl_lval_width(lval);

   string signame(get_renamed_signal(sig));
   vhdl_decl *decl = scope->get_decl(signame);
   if (decl == NULL) {
      // Nothing declared this signal in the scope chain the statement is
      // drawn into: report where it lives rather than trip an assertion
      // the user cannot act on.
      error("assignment to %s (declared in %s, scope type %d) has no VHDL "
            "declaration visible from the process/function that assigns it",
            ivl_signal_name(sig), ivl_scope_name(ivl_signal_scope(sig)),
            (int)ivl_scope_type(ivl_signal_scope(sig)));
      return NULL;
   }

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
   if (decl->get_type()->get_name() == VHDL_TYPE_ARRAY) {
      if (word)
         lval_ref->set_slice(word, 0);
      else if (base)
         lval_ref->set_slice(base, 0);
      // A bit or part-select within the selected word composes after it,
      // and the target is that select: one bit is a scalar, a part a
      // vector of its own width, so the right-hand side is sized to it (it
      // was sized to the whole word: `mem[i][j] = <1-bit expr>' put a
      // vector into one logic3d element, which nvc rejects)
      if (word && base)
         lval_ref->slice_element(base, lval_width - 1);
   }
   else if ((base || word) && ivl_signal_width(sig) > 1)
      lval_ref->set_slice(base ? base : word, lval_width - 1);

   return lval_ref;
}

/*
 * A constant word index or bit offset with an x or z bit: ivl's mark for a
 * constant select it ignores (elab_lval.cc: a word outside the array or an
 * undefined index, "ignoring out of bounds l-value array access" / "ignoring
 * undefined l-value array access"). vvp drops such a store; its translation
 * stored into word 0 (the x constant read as its value bits), so
 * `array1[0] = 1' on `reg array1[2:1]' overwrote array1[1].
 */
static bool const_index_undefined(ivl_expr_t e)
{
   if (e == NULL || ivl_expr_type(e) != IVL_EX_NUMBER)
      return false;
   const char *bits = ivl_expr_bits(e);
   for (unsigned i = 0; i < ivl_expr_width(e); i++)
      if (bits[i] == 'x' || bits[i] == 'z')
         return true;
   return false;
}

static bool lval_store_ignored(ivl_lval_t lval)
{
   return const_index_undefined(ivl_lval_idx(lval))
      || const_index_undefined(ivl_lval_part_off(lval));
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
 * An automatic scope's variables start every activation afresh: x, or 0 for
 * a 2-state one (IEEE 1800 6.21), as vvp's %alloc gives them. The translation
 * keeps one copy -- a task's variables are architecture signals, a block's
 * process variables -- so without this an output the task does not assign
 * in a call returned the last call's value, and a local read before it is
 * written saw it. task: reset only the task's locals and outputs (the call
 * assigns its inputs); else every variable of the block scope. A blocking
 * assignment as draw_assign makes it: a deposit in an initial process, and a
 * blocking target, so a read that follows waits for it.
 */
static void reset_automatic_vars(vhdl_procedural *proc, stmt_container *container,
                                 ivl_scope_t scope, bool task)
{
   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t sig = ivl_scope_sig(scope, i);
      ivl_signal_port_t pt = ivl_signal_port(sig);
      if (task && pt != IVL_SIP_NONE && pt != IVL_SIP_OUTPUT)
         continue;
      if (!seen_signal_before(sig))
         continue;
      vhdl_decl *decl = proc->get_scope()->get_decl(get_renamed_signal(sig));
      if (decl == NULL || decl->get_type() == NULL)
         continue;
      const vhdl_type *t = decl->get_type();
      const char *bit = ivl_signal_data_type(sig) == IVL_VT_BOOL
         ? "L3D_0" : "L3D_X";
      vhdl_expr *v = NULL;
      switch (t->get_name()) {
      case VHDL_TYPE_LOGIC3D:
         v = new vhdl_var_ref(bit, vhdl_type::logic3d());
         break;
      case VHDL_TYPE_LOGIC3D_VECTOR:
         v = new vhdl_var_ref((std::string("(others => ") + bit + ")").c_str(),
                              new vhdl_type(*t));
         break;
      case VHDL_TYPE_REAL:
         v = new vhdl_const_real(0.0);
         break;
      case VHDL_TYPE_INTEGER:
         v = new vhdl_const_int(0);
         break;
      default:
         break;      // an array (memory) local: not reset
      }
      if (v == NULL)
         continue;
      vhdl_var_ref *lhs = new vhdl_var_ref(decl->get_name(), new vhdl_type(*t));
      vhdl_decl::assign_type_t atype = decl->assignment_type();
      if (atype == vhdl_decl::ASSIGN_NONBLOCK) {
         if (deposits_signal(proc, lhs->get_name(), true)) {
            atype = vhdl_decl::ASSIGN_BLOCK;
            proc->mark_deposited(lhs->get_name());
         }
         else
            proc->add_blocking_target(lhs);
      }
      vhdl_abstract_assign_stmt *a = assign_for(atype, lhs, v);
      ostringstream ss;
      ss << "automatic " << ivl_scope_name(scope) << ": a fresh activation";
      a->set_comment(ss.str());
      container->add_stmt(a);
   }
}

/*
 * IVL_ST_ALLOC / IVL_ST_FREE: the start and end of an automatic task's
 * activation, around its call. Tasks are inlined (draw_utask) on one copy of
 * their variables, so ALLOC starts a fresh activation (reset_automatic_vars)
 * and FREE is nothing -- as long as no two activations can overlap: a task
 * called from one process only, never recursively (draw_utask refuses
 * recursion). Calls from two processes have no translation (they would share
 * the one copy, and drive its signals from two processes): a located error.
 */
static std::map<ivl_scope_t, std::pair<ivl_process_t, std::string> > g_auto_task_site;

// A process other than the task's first caller works on its own copy of the
// task's variables: process variables, between the call's ALLOC and FREE
// (the task's variables there are its, as an automatic activation's are).
// The task: the copies its current activation uses
static std::map<ivl_scope_t, std::vector<ivl_signal_t> > g_auto_task_copies;
// (process, variable) -> the name of that process's copy
static std::map<std::pair<vhdl_procedural*, ivl_signal_t>, std::string>
   g_auto_task_copy_names;

static void use_own_task_copies(vhdl_procedural *proc, ivl_scope_t task)
{
   std::vector<ivl_signal_t> &copies = g_auto_task_copies[task];
   for (unsigned i = 0; i < ivl_scope_sigs(task); i++) {
      ivl_signal_t sig = ivl_scope_sig(task, i);
      if (!seen_signal_before(sig))
         continue;
      vhdl_scope *home = find_scope_for_signal(sig);
      vhdl_decl *decl =
         home ? home->get_decl(get_renamed_signal(sig)) : NULL;
      if (decl == NULL || decl->get_type() == NULL)
         continue;
      std::pair<vhdl_procedural*, ivl_signal_t> key(proc, sig);
      std::map<std::pair<vhdl_procedural*, ivl_signal_t>,
               std::string>::iterator nit = g_auto_task_copy_names.find(key);
      std::string name;
      if (nit != g_auto_task_copy_names.end())
         name = nit->second;
      else {
         name = get_renamed_signal(sig) + "_own";
         while (proc->get_scope()->have_declared(name))
            name += "_";
         vhdl_var_decl *v = new vhdl_var_decl(name,
                                              new vhdl_type(*decl->get_type()));
         ostringstream cs;
         cs << "This process's copy of " << ivl_scope_name(task) << "."
            << ivl_signal_basename(sig) << " (automatic)";
         v->set_comment(cs.str());
         proc->get_scope()->add_decl(v);
         g_auto_task_copy_names[key] = name;
      }
      push_signal_home(sig, name, proc->get_scope());
      copies.push_back(sig);
   }
}

static int draw_alloc_free(vhdl_procedural *proc, stmt_container *container,
                           ivl_statement_t stmt)
{
   ivl_scope_t scope = ivl_stmt_call(stmt);
   const bool alloc = ivl_statement_type(stmt) == IVL_ST_ALLOC;
   if (scope == NULL || ivl_scope_type(scope) != IVL_SCT_TASK) {
      error("%s:%d: no VHDL translation for the %s of automatic scope %s",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt),
            alloc ? "allocation" : "release",
            scope ? ivl_scope_name(scope) : "?");
      return 1;
   }
   if (!alloc) {
      // The activation ends: the variables get their homes back
      std::map<ivl_scope_t, std::vector<ivl_signal_t> >::iterator cit =
         g_auto_task_copies.find(scope);
      if (cit != g_auto_task_copies.end()) {
         for (size_t k = 0; k < cit->second.size(); k++)
            pop_signal_home(cit->second[k]);
         g_auto_task_copies.erase(cit);
      }
      return 0;
   }
   ivl_process_t here = get_active_ivl_process();
   ostringstream site;
   site << ivl_stmt_file(stmt) << ":" << ivl_stmt_lineno(stmt);
   std::map<ivl_scope_t, std::pair<ivl_process_t, std::string> >::iterator it =
      g_auto_task_site.find(scope);
   if (it == g_auto_task_site.end())
      g_auto_task_site[scope] = std::make_pair(here, site.str());
   else if (it->second.first != here) {
      // Called from another process too: this activation (and every one of
      // this process) works on its own copy, as Verilog gives each
      // activation its own variables
      if (g_auto_task_copies.count(scope)) {
         error("%s: automatic task %s is entered again before its activation "
               "ends: that has no VHDL translation", site.str().c_str(),
               ivl_scope_name(scope));
         return 1;
      }
      use_own_task_copies(proc, scope);
   }
   reset_automatic_vars(proc, container, scope, true);
   return 0;
}

/*
 * Whether an assignment to signal `name' in `proc' is a deposit (`name := v'
 * -- nvc's --std=2040 deposit on a signal: the new value lands at once, so
 * the process's own later reads see it, and the processes it wakes run in
 * the next delta, after this one suspends) rather than a signal assignment
 * (`name <= v'):
 *  - a time-zero assignment of an initial process: it makes no driver to
 *    conflict with an always block that assigns the signal too;
 *  - any assignment to a signal the process deposited before: nvc drops a
 *    `<=' that follows a `:=' on the same signal;
 *  - a blocking assignment (`blocking') in a process that deposits them all
 *    (vhdl_procedural::deposit_blocking). A `<=' there needed a `wait for
 *    0 ns' before each later read of the signal, and that wait let every
 *    other process run in the middle of this one, where Verilog runs a
 *    process from one suspension to the next without yielding (ivtest
 *    vhdl_test2: the dut, sensitive to `in', ran between `in = in+1' and
 *    `mask = ...' and saw the new `in' with the old `mask').
 */
/*
 * The processes that write each variable (census_writers, a pre-pass over
 * every process before any is drawn): assignment, force and procedural
 * assign targets, the memory of $readmemh/$readmemb, the destination of
 * $swrite* and $sformat, the seed of $random/$urandom/$dist_* and
 * $value$plusargs's variable, in a process's statements and in the tasks it
 * calls.
 */
static std::map<ivl_signal_t, std::set<ivl_process_t> > g_writers;

static void census_write(ivl_expr_t e, ivl_process_t p)
{
   if (e != NULL && ivl_expr_type(e) == IVL_EX_SIGNAL)
      g_writers[ivl_expr_signal(e)].insert(p);
}

static void census_expr(ivl_expr_t e, ivl_process_t p)
{
   if (e == NULL)
      return;
   switch (ivl_expr_type(e)) {
   case IVL_EX_SFUNC:
      {
         const char *n = ivl_expr_name(e);
         if (ivl_expr_parms(e) >= 1
             && (strcmp(n, "$random") == 0 || strcmp(n, "$urandom") == 0
                 || strncmp(n, "$dist_", 6) == 0))
            census_write(ivl_expr_parm(e, 0), p);
         if (ivl_expr_parms(e) >= 2 && strcmp(n, "$value$plusargs") == 0)
            census_write(ivl_expr_parm(e, 1), p);
      }
      // fallthrough
   case IVL_EX_UFUNC:
   case IVL_EX_CONCAT:
      for (unsigned i = 0; i < ivl_expr_parms(e); i++)
         census_expr(ivl_expr_parm(e, i), p);
      break;
   case IVL_EX_BINARY:
   case IVL_EX_SELECT:
      census_expr(ivl_expr_oper1(e), p);
      census_expr(ivl_expr_oper2(e), p);
      break;
   case IVL_EX_UNARY:
      census_expr(ivl_expr_oper1(e), p);
      break;
   case IVL_EX_TERNARY:
      census_expr(ivl_expr_oper1(e), p);
      census_expr(ivl_expr_oper2(e), p);
      census_expr(ivl_expr_oper3(e), p);
      break;
   default:
      break;
   }
}

static void census_stmt(ivl_statement_t s, ivl_process_t p,
                        std::set<ivl_scope_t> &tasks)
{
   if (s == NULL)
      return;
   switch (ivl_statement_type(s)) {
   case IVL_ST_ASSIGN:
   case IVL_ST_ASSIGN_NB:
   case IVL_ST_CASSIGN:
   case IVL_ST_DEASSIGN:
   case IVL_ST_FORCE:
   case IVL_ST_RELEASE:
      for (unsigned i = 0; i < ivl_stmt_lvals(s); i++)
         if (ivl_signal_t sig = ivl_lval_sig(ivl_stmt_lval(s, i)))
            g_writers[sig].insert(p);
      if (ivl_statement_type(s) != IVL_ST_DEASSIGN
          && ivl_statement_type(s) != IVL_ST_RELEASE)
         census_expr(ivl_stmt_rval(s), p);
      break;
   case IVL_ST_BLOCK:
   case IVL_ST_FORK:
   case IVL_ST_FORK_JOIN_ANY:
   case IVL_ST_FORK_JOIN_NONE:
      for (unsigned i = 0; i < ivl_stmt_block_count(s); i++)
         census_stmt(ivl_stmt_block_stmt(s, i), p, tasks);
      break;
   case IVL_ST_CONDIT:
      census_expr(ivl_stmt_cond_expr(s), p);
      census_stmt(ivl_stmt_cond_true(s), p, tasks);
      census_stmt(ivl_stmt_cond_false(s), p, tasks);
      break;
   case IVL_ST_CASE:
   case IVL_ST_CASER:
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
      census_expr(ivl_stmt_cond_expr(s), p);
      for (unsigned i = 0; i < ivl_stmt_case_count(s); i++)
         census_stmt(ivl_stmt_case_stmt(s, i), p, tasks);
      break;
   case IVL_ST_FORLOOP:
      census_stmt(ivl_stmt_init_stmt(s), p, tasks);
      census_expr(ivl_stmt_cond_expr(s), p);
      census_stmt(ivl_stmt_step_stmt(s), p, tasks);
      census_stmt(ivl_stmt_sub_stmt(s), p, tasks);
      break;
   case IVL_ST_WHILE:
   case IVL_ST_DO_WHILE:
   case IVL_ST_REPEAT:
      census_expr(ivl_stmt_cond_expr(s), p);
      census_stmt(ivl_stmt_sub_stmt(s), p, tasks);
      break;
   case IVL_ST_FOREVER:
   case IVL_ST_DELAY:
   case IVL_ST_DELAYX:
   case IVL_ST_WAIT:
      census_stmt(ivl_stmt_sub_stmt(s), p, tasks);
      break;
   case IVL_ST_UTASK:
      {
         ivl_scope_t t = ivl_stmt_call(s);
         if (t != NULL && tasks.insert(t).second)
            census_stmt(ivl_scope_def(t), p, tasks);
      }
      break;
   case IVL_ST_STASK:
      {
         const char *n = ivl_stmt_name(s);
         const unsigned np = ivl_stmt_parm_count(s);
         if ((strcmp(n, "$readmemh") == 0 || strcmp(n, "$readmemb") == 0)
             && np >= 2)
            census_write(ivl_stmt_parm(s, 1), p);
         if ((strncmp(n, "$swrite", 7) == 0 || strcmp(n, "$sformat") == 0)
             && np >= 1)
            census_write(ivl_stmt_parm(s, 0), p);
         if ((strcmp(n, "$random") == 0 || strcmp(n, "$urandom") == 0)
             && np >= 1)
            census_write(ivl_stmt_parm(s, 0), p);
         for (unsigned i = 0; i < np; i++)
            census_expr(ivl_stmt_parm(s, i), p);
      }
      break;
   default:
      break;
   }
}

extern "C" int census_writers(ivl_process_t p, void *)
{
   std::set<ivl_scope_t> tasks;
   census_stmt(ivl_process_stmt(p), p, tasks);
   return 0;
}

// The process being drawn is the only one that writes `sig' (census_writers)
static bool sole_writer(ivl_signal_t sig)
{
   ivl_process_t here = get_active_ivl_process();
   std::map<ivl_signal_t, std::set<ivl_process_t> >::const_iterator it =
      g_writers.find(sig);
   return here != NULL && it != g_writers.end() && it->second.size() == 1
      && *it->second.begin() == here;
}

static bool deposits_signal(vhdl_procedural *proc, const std::string &name,
                            bool blocking, ivl_signal_t sig)
{
   // A nonblocking assignment of the variable's only writer is a `<=' (one
   // VHDL driver, no conflict), landing in the next delta as any NBA does.
   // It was a deposit in an initial block (before the end of its first
   // delay's statement, after an event control, or once the block had
   // deposited the variable), which lands at once: the block's own later
   // reads in the time step (`w <= 7; $display(w)' printed 7, vvp the old
   // value) and what ran after it in the same delta saw the new value. nvc
   // applies the `<=' although an earlier deposit made the signal differ
   // from the driver (rt/model.c sched_driver). With other writers it stays
   // a deposit (two drivers of an unresolved signal: the first one's value
   // wins). (A block that a deposited clock wakes at the same time also
   // runs in that next delta; the round-6 repair's probes of a stimulus
   // `<=' at a clock edge, in every order of clock, stimulus and flop,
   // sample what vvp samples. nvc's Verilog NBA region, its nonblocking
   // sched_deposit, has no VHDL form.)
   if (!blocking && sig != NULL && proc->deposit_blocking() && sole_writer(sig))
      return false;
   return proc->get_scope()->initializing() || proc->was_deposited(name)
      || (blocking && proc->deposit_blocking());
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
   // A NULL proc means we are building a companion (postponed) process for
   // $monitor/$strobe, which reads settled values by construction.
   if (proc == NULL)
      return;

   vhdl_var_set_t read;
   expr->find_vars(read);

   // A read of a blocking target this process assigns with `<='. (A deposit
   // is read back at once and registers no target, so no wait is made for
   // it: a process waiting on the deposited variable -- a VHDL dut's
   // process(input) -- no longer runs in the middle of this one, and a net
   // fed by it is not updated before the process suspends, as in Verilog:
   // ivtest sched2.)
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

// True if evaluating `e' reads a net (wire, tri, tri0/tri1, an input
// port...): only a net value can carry a pull or weak strength.
static bool expr_reads_net(ivl_expr_t e)
{
   if (e == NULL)
      return false;
   switch (ivl_expr_type(e)) {
   case IVL_EX_SIGNAL:
      return ivl_signal_type(ivl_expr_signal(e)) != IVL_SIT_REG;
   case IVL_EX_SELECT:
   case IVL_EX_BINARY:
      return expr_reads_net(ivl_expr_oper1(e))
         || expr_reads_net(ivl_expr_oper2(e));
   case IVL_EX_UNARY:
      return expr_reads_net(ivl_expr_oper1(e));
   case IVL_EX_TERNARY:
      return expr_reads_net(ivl_expr_oper2(e))
         || expr_reads_net(ivl_expr_oper3(e));
   case IVL_EX_CONCAT:
      for (unsigned i = 0; i < ivl_expr_parms(e); i++)
         if (expr_reads_net(ivl_expr_parm(e, i)))
            return true;
      return false;
   default:
      return false;
   }
}

// A Verilog variable holds a 4-state value and no strength: a read of a
// net that a pull, a tri1/tri0 or an AMS BIDIR A2D drives weakly
// (L3D_H/L/W) is stored strong, so the variable reads -- and drives,
// through a continuous assignment -- like Verilog's (H -> 1, L -> 0, W ->
// X; l3d_strengthen keeps Z, X and U).  Identity on strong values.  A
// vector bit by bit (`data = pads' with BIDIR pads kept L3D_H/L bits,
// which lost to a strong driver of a net the variable drives).
static vhdl_expr *variable_value(vhdl_expr *rhs, ivl_expr_t src)
{
   if (!get_sv2vhdl_mode() || rhs == NULL || rhs->get_type() == NULL
       || (rhs->get_type()->get_name() != VHDL_TYPE_LOGIC3D
           && rhs->get_type()->get_name() != VHDL_TYPE_LOGIC3D_VECTOR)
       || !expr_reads_net(src))
      return rhs;
   vhdl_fcall *f = new vhdl_fcall("l3d_strengthen", new vhdl_type(*rhs->get_type()));
   f->add_expr(rhs);
   return f;
}

/*
 * Read a run-time index once, at the statement, into a fresh process
 * variable <prefix><n>, and open `if <n> >= lo and <n> <= hi then': what is
 * drawn into the returned container stores only for an index inside lo..hi
 * (a store outside is dropped, as vvp drops it; a VHDL index stops the run
 * there). *ref: a reference to the variable, for the store's select.
 */
static stmt_container *guard_index(vhdl_procedural *proc,
                                   stmt_container *container,
                                   vhdl_expr *index, int lo, int hi,
                                   const char *prefix, const char *comment,
                                   vhdl_var_ref **ref)
{
   static int guard_count = 0;
   ostringstream ix;
   ix << prefix << guard_count++;
   proc->get_scope()->add_decl(new vhdl_var_decl(ix.str(), vhdl_type::integer()));
   vhdl_assign_stmt *capture = new vhdl_assign_stmt(
      new vhdl_var_ref(ix.str(), vhdl_type::integer()), index);
   capture->set_comment(comment);
   container->add_stmt(capture);
   vhdl_binop_expr *in_range =
      new vhdl_binop_expr(VHDL_BINOP_AND, vhdl_type::boolean());
   in_range->add_expr(new vhdl_binop_expr(
      new vhdl_var_ref(ix.str(), vhdl_type::integer()),
      VHDL_BINOP_GEQ, new vhdl_const_int(lo), vhdl_type::boolean()));
   in_range->add_expr(new vhdl_binop_expr(
      new vhdl_var_ref(ix.str(), vhdl_type::integer()),
      VHDL_BINOP_LEQ, new vhdl_const_int(hi), vhdl_type::boolean()));
   vhdl_if_stmt *guard = new vhdl_if_stmt(in_range);
   container->add_stmt(guard);
   *ref = new vhdl_var_ref(ix.str(), vhdl_type::integer());
   return guard->get_then_container();
}

/*
 * The store of the w-bit value `rhs' (w > 1) at the run-time bit offset
 * `offset' of a vector whose bits run lo..hi (a vector signal, or a memory
 * word): each bit lands only inside the vector, as Verilog stores it (a VHDL
 * slice stopped the run), with the assignment's intra-assignment delay:
 *    OOB_WriteV_Tmp_<n> := <rhs>;   OOB_WriteV_Idx_<n> := <offset>;
 *    if OOB_WriteV_Idx_<n> >= lo - (w-1) and OOB_WriteV_Idx_<n> <= hi then
 *       for OOB_P in 0 to w-1 loop
 *          if <idx> + OOB_P >= lo and <idx> + OOB_P <= hi then
 *             <bit(idx + OOB_P)> := | <= OOB_WriteV_Tmp_<n>(OOB_P) [after d];
 *          end if;
 *       end loop;
 *    end if;
 * (the outer test keeps idx + OOB_P from overflowing INTEGER for an extreme
 * index). make_bit(pos): a fresh reference to the target bit at offset pos;
 * whole: a reference to the whole target signal. Deposit or assign as
 * make_assignment does for a whole target; a delayed store keeps its delay
 * (it was deposited at once: `v[k +: 2] <= #3 v2' landed 3 time units early).
 * Returns false when the store cannot be drawn here.
 */
static bool draw_runtime_part_store(vhdl_procedural *proc,
                                    stmt_container *container,
                                    ivl_statement_t stmt, bool emul_blocking,
                                    vhdl_decl *decl, vhdl_var_ref *whole,
                                    vhdl_expr *rhs, vhdl_expr *offset,
                                    int w, int lo, int hi,
                                    std::function<vhdl_var_ref *(vhdl_expr *)>
                                       make_bit)
{
   vhdl_expr *after = NULL;
   if (ivl_expr_t i_delay = ivl_stmt_delay_expr(stmt)) {
      if ((after = translate_time_expr(i_delay)) == NULL)
         return false;
      emit_wait_for_0(proc, container, stmt, after);
   }

   vhdl_decl::assign_type_t at = decl->assignment_type();
   if (at == vhdl_decl::ASSIGN_NONBLOCK && after == NULL
       && deposits_signal(proc, whole->get_name(), emul_blocking,
                          ivl_lval_sig(ivl_stmt_lval(stmt, 0)))) {
      at = vhdl_decl::ASSIGN_BLOCK;
      proc->mark_deposited(whole->get_name());
   }
   if (!check_valid_assignment(at, proc, stmt))
      return false;
   if (at == vhdl_decl::ASSIGN_NONBLOCK && emul_blocking)
      proc->add_blocking_target(whole);

   static int oobv_count = 0;
   ostringstream tn, ix;
   tn << "OOB_WriteV_Tmp_" << oobv_count;
   ix << "OOB_WriteV_Idx_" << oobv_count++;
   vhdl_type lvw(VHDL_TYPE_LOGIC3D_VECTOR, w - 1, 0);
   vhdl_var_decl *td = new vhdl_var_decl(
      tn.str(), vhdl_type::logic3d_vector(w - 1, 0));
   proc->get_scope()->add_decl(td);
   vhdl_var_decl *xd = new vhdl_var_decl(ix.str(), vhdl_type::integer());
   proc->get_scope()->add_decl(xd);
   container->add_stmt(new vhdl_assign_stmt(
      td->make_ref(), variable_value(rhs->cast(&lvw), ivl_stmt_rval(stmt))));
   container->add_stmt(new vhdl_assign_stmt(xd->make_ref(), offset));

   vhdl_binop_expr *outer = new vhdl_binop_expr(
      new vhdl_binop_expr(
         new vhdl_var_ref(ix.str().c_str(), vhdl_type::integer()),
         VHDL_BINOP_GEQ, new vhdl_const_int(lo - (w - 1)),
         vhdl_type::boolean()),
      VHDL_BINOP_AND,
      new vhdl_binop_expr(
         new vhdl_var_ref(ix.str().c_str(), vhdl_type::integer()),
         VHDL_BINOP_LEQ, new vhdl_const_int(hi), vhdl_type::boolean()),
      vhdl_type::boolean());
   vhdl_if_stmt *outer_if = new vhdl_if_stmt(outer);

   vhdl_for_stmt *loop = new vhdl_for_stmt("OOB_P",
      new vhdl_const_int(0), new vhdl_const_int(w - 1));
   vhdl_expr *pos = new vhdl_binop_expr(
      new vhdl_var_ref(ix.str().c_str(), vhdl_type::integer()),
      VHDL_BINOP_ADD, new vhdl_var_ref("OOB_P", vhdl_type::integer()),
      vhdl_type::integer());
   vhdl_binop_expr *guard = new vhdl_binop_expr(
      new vhdl_binop_expr(pos, VHDL_BINOP_GEQ, new vhdl_const_int(lo),
                          vhdl_type::boolean()),
      VHDL_BINOP_AND,
      new vhdl_binop_expr(
         new vhdl_binop_expr(
            new vhdl_var_ref(ix.str().c_str(), vhdl_type::integer()),
            VHDL_BINOP_ADD, new vhdl_var_ref("OOB_P", vhdl_type::integer()),
            vhdl_type::integer()),
         VHDL_BINOP_LEQ, new vhdl_const_int(hi), vhdl_type::boolean()),
      vhdl_type::boolean());
   vhdl_if_stmt *iff = new vhdl_if_stmt(guard);
   vhdl_var_ref *bit_lhs = make_bit(new vhdl_binop_expr(
      new vhdl_var_ref(ix.str().c_str(), vhdl_type::integer()),
      VHDL_BINOP_ADD, new vhdl_var_ref("OOB_P", vhdl_type::integer()),
      vhdl_type::integer()));
   vhdl_var_ref *bit_rhs = new vhdl_var_ref(
      tn.str(), vhdl_type::logic3d_vector(w - 1, 0));
   bit_rhs->set_slice(new vhdl_var_ref("OOB_P", vhdl_type::integer()));
   vhdl_abstract_assign_stmt *a = assign_for(at, bit_lhs, bit_rhs);
   if (after)
      a->set_after(after);
   iff->get_then_container()->add_stmt(a);
   loop->get_container()->add_stmt(iff);
   outer_if->get_then_container()->add_stmt(loop);
   container->add_stmt(outer_if);
   return true;
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
   // The ternary if/else expansion below is an idiom optimization and only
   // correct for a plain unsliced lvalue: the clamp and dynamic-index
   // guarded-write paths emit a single RHS expression and return early, so
   // a pre-split ternary reached them as its bare TRUE-ARM -- condition and
   // else-arm silently dropped (VeeR-EH2: every icache fill wrote the
   // never-driven debug-write bus instead of debug?debug:bank, zeroing the
   // banks). A slice-targeted ternary translates as an ordinary Ternary
   // expression instead.
   const bool plain_lval = lvals.size() == 1
      && lvals.front()->get_slice() == NULL
      && lvals.front()->extra_range_width() <= 0;
   if (ivl_expr_type(rval) == IVL_EX_TERNARY && plain_lval) {
      begin_conditional_eval();   // one branch runs (translate_ternary)
      rhs = translate_expr(ivl_expr_oper2(rval));
      rhs2 = translate_expr(ivl_expr_oper3(rval));
      end_conditional_eval();
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
      bool shift = false;
      switch (comp_op) {
      case '+': binop = VHDL_BINOP_ADD; break;
      case '-': binop = VHDL_BINOP_SUB; break;
      case '*': binop = VHDL_BINOP_MULT; break;
      case '/': binop = VHDL_BINOP_DIV; break;
      case '%': binop = VHDL_BINOP_MOD; break;
      case '&': binop = VHDL_BINOP_AND; break;
      case '|': binop = VHDL_BINOP_OR; break;
      case '^': binop = VHDL_BINOP_XOR; break;
      case 'l': binop = VHDL_BINOP_SL; shift = true; break;
      case 'r': binop = VHDL_BINOP_SR; shift = true; break;
      case 'R':    // >>>=: arithmetic on a signed target only, as for >>>
         binop = ivl_signal_signed(ivl_lval_sig(ivl_stmt_lval(stmt, 0)))
            ? VHDL_BINOP_SRA : VHDL_BINOP_SR;
         shift = true;
         break;
      default:
         // Never a silent stand-in operator
         error("%s:%d: no VHDL translation for the compressed assignment "
               "operator '%c='", ivl_stmt_file(stmt), ivl_stmt_lineno(stmt),
               comp_op);
         return;
      }
      // Build: lhs <op> rhs
      vhdl_var_ref *lhs_read =
         make_assign_lhs(ivl_stmt_lval(stmt, 0), proc->get_scope());
      if (shift && lhs_read->get_type() != NULL) {
         // The shift count is self-determined: an integer (l3d_shcount for
         // a vector, as translate_shift does), never width-merged with the
         // shifted operand
         vhdl_expr *count;
         if (rhs->get_type()
             && rhs->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
            vhdl_fcall *sc = new vhdl_fcall("l3d_shcount", vhdl_type::integer());
            sc->add_expr(rhs);
            count = sc;
         }
         else {
            vhdl_type integer(VHDL_TYPE_INTEGER);
            count = rhs->cast(&integer);
         }
         const vhdl_type *lt = new vhdl_type(*lhs_read->get_type());
         if (binop == VHDL_BINOP_SRA) {
            vhdl_fcall *sra = new vhdl_fcall(
               lt->get_name() == VHDL_TYPE_LOGIC3D_VECTOR ? "l3d_sra"
                                                          : "shift_right", lt);
            sra->add_expr(lhs_read);
            sra->add_expr(count);
            rhs = sra;
         }
         else
            rhs = new vhdl_binop_expr(lhs_read, binop, count, lt);
      }
      else {
         // The implicit read joins Verilog width propagation like any operand:
         // a scalar meeting a wider RHS (e.g. `bit |= 5'h01 & ...`) computes at
         // the wide width and the assignment cast truncates back to the LHS.
         // Mirrors translate_binary's operand normalization, which this
         // hand-built binop otherwise bypasses.
         vhdl_expr *lhs_x = lhs_read;
         if (get_sv2vhdl_mode() && lhs_x->get_type() && rhs->get_type()) {
            vhdl_type_name_t lt = lhs_x->get_type()->get_name();
            vhdl_type_name_t rt = rhs->get_type()->get_name();
            if (lt == VHDL_TYPE_LOGIC3D && rt == VHDL_TYPE_LOGIC3D_VECTOR)
               lhs_x = lhs_x->cast(rhs->get_type());
            else if (rt == VHDL_TYPE_LOGIC3D && lt == VHDL_TYPE_LOGIC3D_VECTOR)
               rhs = rhs->cast(lhs_x->get_type());
         }
         // A signed /= or %= (a whole signed target, a signed right-hand
         // side) divides signed, as translate_binary's / and % do: the
         // logic3d_vector "/" and "mod" are unsigned.
         ivl_lval_t lv0 = ivl_stmt_lval(stmt, 0);
         ivl_signal_t ls0 = ivl_lval_sig(lv0);
         if (get_sv2vhdl_mode()
             && (binop == VHDL_BINOP_DIV || binop == VHDL_BINOP_MOD)
             && ls0 != NULL && ivl_signal_signed(ls0) && ivl_expr_signed(rval)
             && ivl_lval_part_off(lv0) == NULL
             && ivl_lval_width(lv0) == ivl_signal_width(ls0)
             && lhs_x->get_type() && rhs->get_type()
             && lhs_x->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR
             && rhs->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
            const char *fn = "l3d_div_s";
            if (binop == VHDL_BINOP_MOD) {
               require_support_function(SF_REM_SIGNED);
               fn = support_function::function_name(SF_REM_SIGNED);
            }
            vhdl_fcall *f = new vhdl_fcall(fn, new vhdl_type(*lhs_x->get_type()));
            f->add_expr(lhs_x);
            f->add_expr(rhs);
            rhs = f;
         }
         else
            rhs = new vhdl_binop_expr(lhs_x, binop, rhs,
                                      new vhdl_type(*lhs_x->get_type()));
      }
   }

   emit_wait_for_0(proc, container, stmt, rhs);
   if (rhs2)
      emit_wait_for_0(proc, container, stmt, rhs2);

   if (lvals.size() == 1) {
      // A constant select ivl ignores (const_index_undefined): the store is
      // dropped, as vvp drops it; the right-hand side was evaluated above,
      // as vvp evaluates it
      if (lval_store_ignored(ivl_stmt_lval(stmt, 0)))
         return;

      vhdl_var_ref *lhs = lvals.front();
      bool clamped = false;

      // sv2vhdl: a statically out-of-range part-select WRITE drops the
      // out-of-range bits (a VHDL slice raises a bounds error instead).
      // Clamp the target range and take the matching sub-slice of the RHS
      // through a temporary.
      // NB: set_slice mutates the ref's type to the slice's own width, so the
      // target bounds must come from the DECLARATION, not lhs->get_type().
      vhdl_decl *lhs_decl = proc->get_scope()->get_decl(lhs->get_name());

      // Static out-of-range ARRAY WORD write: a Verilog OOB word store is a
      // no-op (VHDL raises a bounds error). Also give a 1-element array a
      // word index when assigned a plain vector (x[0:0] = v).
      if (get_sv2vhdl_mode() && lhs_decl && lhs_decl->get_type()
          && lhs_decl->get_type()->get_name() == VHDL_TYPE_ARRAY) {
         const int wlo = std::min(lhs_decl->get_type()->get_lsb(),
                                  lhs_decl->get_type()->get_msb());
         const int whi = std::max(lhs_decl->get_type()->get_lsb(),
                                  lhs_decl->get_type()->get_msb());
         if (lhs->get_slice() != NULL) {
            vhdl_const_int *wb =
               dynamic_cast<vhdl_const_int*>(lhs->get_slice());
            if (wb && (wb->get_value() < wlo || wb->get_value() > whi))
               return;      // a word (or part of one) out of range: lost
         }
         else if (lhs->get_slice() == NULL && wlo == whi
                  && rhs->get_type()
                  && rhs->get_type()->get_name()
                        == VHDL_TYPE_LOGIC3D_VECTOR) {
            // Single-element array assigned a vector: target the element
            lhs->set_slice(new vhdl_const_int(wlo), 0);
         }

         // A store at a run-time word index outside the array is dropped,
         // as vvp drops it, where a VHDL index stops the run. The index is
         // read once, at the statement, as Verilog does:
         //    OOB_WIdx_<n> := <index>;
         //    if OOB_WIdx_<n> >= lo and OOB_WIdx_<n> <= hi then
         //       <the store, at word OOB_WIdx_<n>>
         //    end if;
         if (lhs->get_slice() != NULL
             && dynamic_cast<vhdl_const_int*>(lhs->get_slice()) == NULL) {
            static int oob_widx_count = 0;
            ostringstream ix;
            ix << "OOB_WIdx_" << oob_widx_count++;
            proc->get_scope()->add_decl(
               new vhdl_var_decl(ix.str(), vhdl_type::integer()));
            vhdl_assign_stmt *capture = new vhdl_assign_stmt(
               new vhdl_var_ref(ix.str(), vhdl_type::integer()),
               lhs->get_slice());
            capture->set_comment("A store outside the array is dropped");
            container->add_stmt(capture);
            lhs->replace_slice(new vhdl_var_ref(ix.str(), vhdl_type::integer()));
            vhdl_binop_expr *in_range =
               new vhdl_binop_expr(VHDL_BINOP_AND, vhdl_type::boolean());
            in_range->add_expr(new vhdl_binop_expr(
               new vhdl_var_ref(ix.str(), vhdl_type::integer()),
               VHDL_BINOP_GEQ, new vhdl_const_int(wlo), vhdl_type::boolean()));
            in_range->add_expr(new vhdl_binop_expr(
               new vhdl_var_ref(ix.str(), vhdl_type::integer()),
               VHDL_BINOP_LEQ, new vhdl_const_int(whi), vhdl_type::boolean()));
            vhdl_if_stmt *guard = new vhdl_if_stmt(in_range);
            container->add_stmt(guard);
            container = guard->get_then_container();
         }
      }

      // Array-word + part-select lvalue (mem(word)(range)): clamp a static
      // OOB part against the ELEMENT type's bounds by rewriting the extra
      // range slice (the plain-vector clamp below can't see it).
      if (get_sv2vhdl_mode() && lhs_decl && lhs_decl->get_type()
          && lhs_decl->get_type()->get_name() == VHDL_TYPE_ARRAY
          && lhs->extra_range_width() > 0
          && lhs_decl->get_type()->get_base()
          && lhs_decl->get_type()->get_base()->get_name()
                == VHDL_TYPE_LOGIC3D_VECTOR) {
         vhdl_const_int *eb =
            dynamic_cast<vhdl_const_int*>(lhs->last_extra_base());
         if (eb) {
            const vhdl_type *et = lhs_decl->get_type()->get_base();
            const int lo_e = et->get_lsb();
            const int hi_e = et->get_msb();
            const int b = eb->get_value();
            const int w = lhs->extra_range_width() + 1;
            if (b < lo_e || b + w - 1 > hi_e) {
               const int clo = b > lo_e ? b : lo_e;
               const int chi = (b + w - 1) < hi_e ? (b + w - 1) : hi_e;
               if (clo > chi)
                  return;      // whole part out of range: write lost
               static int oob_aw_count = 0;
               ostringstream tn;
               tn << "OOB_AWrite_Tmp_" << oob_aw_count++;
               vhdl_type lvw(VHDL_TYPE_LOGIC3D_VECTOR, w - 1, 0);
               vhdl_var_decl *td = new vhdl_var_decl(
                  tn.str(), vhdl_type::logic3d_vector(w - 1, 0));
               proc->get_scope()->add_decl(td);
               container->add_stmt(
                  new vhdl_assign_stmt(td->make_ref(), rhs->cast(&lvw)));
               lhs->set_last_extra(new vhdl_const_int(clo), chi - clo);
               vhdl_var_ref *tr = new vhdl_var_ref(
                  tn.str(), vhdl_type::logic3d_vector(w - 1, 0));
               tr->set_slice(new vhdl_const_int(clo - b), chi - clo);
               rhs = tr;
               clamped = true;
            }
         }
      }

      // A bit or a part of a memory word at a run-time offset (`m[i][k] = b',
      // `m[i][k +: 4] = v'): the bits outside the word are dropped, as vvp
      // drops them (`m(OOB_WIdx_0)(l3d_index(k, True)) := ...' stopped the
      // run). A bit: under guard_index; a part: bit by bit
      // (draw_runtime_part_store).
      if (get_sv2vhdl_mode() && lhs_decl && lhs_decl->get_type()
          && lhs_decl->get_type()->get_name() == VHDL_TYPE_ARRAY
          && lhs->get_slice() != NULL
          && lhs->extra_range_width() >= 0
          && dynamic_cast<vhdl_const_int*>(lhs->last_extra_base()) == NULL
          && lhs_decl->get_type()->get_base()
          && lhs_decl->get_type()->get_base()->get_name()
                == VHDL_TYPE_LOGIC3D_VECTOR) {
         const vhdl_type *et = lhs_decl->get_type()->get_base();
         const int lo_e = std::min(et->get_lsb(), et->get_msb());
         const int hi_e = std::max(et->get_lsb(), et->get_msb());
         if (lhs->extra_range_width() == 0) {
            vhdl_var_ref *ixr = NULL;
            container = guard_index(proc, container, lhs->last_extra_base(),
                                    lo_e, hi_e, "OOB_EIdx_",
                                    "A bit outside the word is dropped", &ixr);
            lhs->set_last_extra(ixr, 0);
         }
         else {
            // The word's index: a constant, or guard_index's variable
            vhdl_expr *word = lhs->get_slice();
            vhdl_const_int *cw = dynamic_cast<vhdl_const_int*>(word);
            vhdl_var_ref *vw = dynamic_cast<vhdl_var_ref*>(word);
            if (cw == NULL && vw == NULL) {
               error("%s:%d: no VHDL translation for this store into a part "
                     "of a memory word", ivl_stmt_file(stmt),
                     ivl_stmt_lineno(stmt));
               return;
            }
            const int cwv = cw ? cw->get_value() : 0;
            const std::string vwn = vw ? vw->get_name() : std::string();
            const std::string name = lhs->get_name();
            const vhdl_type *atype = lhs_decl->get_type();
            vhdl_var_ref *whole = new vhdl_var_ref(name, new vhdl_type(*atype));
            if (!draw_runtime_part_store(
                   proc, container, stmt, emul_blocking, lhs_decl, whole, rhs,
                   lhs->last_extra_base(), lhs->extra_range_width() + 1,
                   lo_e, hi_e,
                   [&](vhdl_expr *pos) {
                      vhdl_var_ref *r = new vhdl_var_ref(name,
                                                         new vhdl_type(*atype));
                      if (cw)
                         r->set_slice(new vhdl_const_int(cwv), 0);
                      else
                         r->set_slice(new vhdl_var_ref(vwn,
                                                       vhdl_type::integer()), 0);
                      r->slice_element(pos, 0);
                      return r;
                   }))
               return;
            return;
         }
      }

      if (get_sv2vhdl_mode() && lhs->get_slice() && lhs_decl
          && lhs_decl->get_type()
          && lhs_decl->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
         vhdl_const_int *cb = dynamic_cast<vhdl_const_int*>(lhs->get_slice());
         const int lo_t = lhs_decl->get_type()->get_lsb();
         const int hi_t = lhs_decl->get_type()->get_msb();
         if (cb == NULL && lhs->get_slice_width() == 0) {
            // A bit-select store at a run-time offset outside the vector is
            // dropped, as vvp drops it (`v(l3d_index(k, True)) := L3D_1'
            // stopped the run: "index 6 outside of INTEGER range 3 downto
            // 0"); the rest of the store is drawn as usual, under the guard
            vhdl_var_ref *ixr = NULL;
            container = guard_index(proc, container, lhs->get_slice(),
                                    std::min(lo_t, hi_t), std::max(lo_t, hi_t),
                                    "OOB_BIdx_",
                                    "A bit outside the vector is dropped", &ixr);
            lhs->replace_slice(ixr);
         }
         if (cb == NULL && lhs->get_slice_width() > 0) {
            // Runtime-variable part-select write: any bit can be out of
            // range, and Verilog silently drops those: bit by bit, under a
            // bounds guard (draw_runtime_part_store)
            const std::string name = lhs->get_name();
            const vhdl_type *vtype = lhs_decl->get_type();
            vhdl_var_ref *whole = new vhdl_var_ref(name, new vhdl_type(*vtype));
            draw_runtime_part_store(
               proc, container, stmt, emul_blocking, lhs_decl, whole, rhs,
               lhs->get_slice(), lhs->get_slice_width() + 1,
               std::min(lo_t, hi_t), std::max(lo_t, hi_t),
               [&](vhdl_expr *pos) {
                  vhdl_var_ref *r = new vhdl_var_ref(name, new vhdl_type(*vtype));
                  r->set_slice(pos);
                  return r;
               });
            return;
         }
         if (cb) {
            const int b = cb->get_value();
            const int w = lhs->get_slice_width() + 1;
            if (b < lo_t || b + w - 1 > hi_t) {
               const int clo = b > lo_t ? b : lo_t;
               const int chi = (b + w - 1) < hi_t ? (b + w - 1) : hi_t;
               if (clo > chi)
                  return;      // entirely out of range: the write is lost
               static int oob_tmp_count = 0;
               ostringstream tn;
               tn << "OOB_Write_Tmp_" << oob_tmp_count++;
               vhdl_type lvw(VHDL_TYPE_LOGIC3D_VECTOR, w - 1, 0);
               vhdl_var_decl *td = new vhdl_var_decl(
                  tn.str(), vhdl_type::logic3d_vector(w - 1, 0));
               proc->get_scope()->add_decl(td);
               container->add_stmt(
                  new vhdl_assign_stmt(td->make_ref(), rhs->cast(&lvw)));
               lhs->set_slice(new vhdl_const_int(clo), chi - clo);
               vhdl_var_ref *tr = new vhdl_var_ref(
                  tn.str(), vhdl_type::logic3d_vector(w - 1, 0));
               tr->set_slice(new vhdl_const_int(clo - b), chi - clo);
               rhs = tr;
               clamped = true;
            }
         }
      }

      if (!clamped) {
         // A word+part lvalue (mem(word)(part-range)) has the ARRAY type on
         // the ref; the assignment target is really the part's width.
         const int xw = lhs->extra_range_width();
         if (get_sv2vhdl_mode() && xw > 0) {
            vhdl_type evw(VHDL_TYPE_LOGIC3D_VECTOR, xw, 0);
            rhs = rhs->cast(&evw);
         }
         else
            rhs = rhs->cast(lhs->get_type());
         rhs = variable_value(rhs, rval);
      }

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

      // A signal target is deposited (:=) instead of assigned (<=) where
      // deposits_signal says so: in an initial process at time zero (no
      // VHDL driver to conflict with an always process driving the same
      // signal: a deposit writes the effective value without a driver,
      // matching Verilog's shared-driver reg semantics), once the process
      // deposited the signal before (nvc drops a later <= on a signal that
      // was already assigned with :=), and for every blocking assignment of
      // a deposit_blocking process. NVC --std=2040 supports := on signals
      // (T_DEPOSIT).
      //
      // Exception: an NBA with an `after` delay (`a <= #2 1;`) needs the
      // signal-assignment semantics so the value change is scheduled,
      // not deposited immediately.  vhdl_assign_stmt has no `after`
      // form, so keep it as a non-blocking signal assignment.
      vhdl_decl::assign_type_t atype = decl->assignment_type();
      if (atype == vhdl_decl::ASSIGN_NONBLOCK && after == NULL
          && deposits_signal(proc, lhs->get_name(), emul_blocking,
                             ivl_lval_sig(ivl_stmt_lval(stmt, 0)))) {
         atype = vhdl_decl::ASSIGN_BLOCK;
         proc->mark_deposited(lhs->get_name());
      }

      // A blocking <= to a signal: a later read in this process must first
      // let it land (emit_wait_for_0), or the shadow pass (process.cc)
      // gives it a variable. A deposit is read back at once.
      if (atype == vhdl_decl::ASSIGN_NONBLOCK && emul_blocking)
          proc->add_blocking_target(lhs);

      // A small optimisation is to expand ternary RHSs into an
      // if statement (eliminates a function call and produces
      // more idiomatic code)
      if (ivl_expr_type(rval) == IVL_EX_TERNARY && rhs2 != NULL) {
         rhs2 = variable_value(rhs2->cast(lhs->get_type()), rval);
         vhdl_var_ref *lhs2 =
            make_assign_lhs(ivl_stmt_lval(stmt, 0), proc->get_scope());

         vhdl_expr *test = translate_expr(ivl_expr_oper1(rval));
         if (NULL == test)
            return;

         emit_wait_for_0(proc, container, stmt, test);

         if (!check_valid_assignment(atype, proc, stmt))
            return;

         vhdl_if_stmt *vhdif = new vhdl_if_stmt(test);

         // True part
         {
            vhdl_abstract_assign_stmt *a = assign_for(atype, lhs, rhs);
            if (after)
               a->set_after(after);
            vhdif->get_then_container()->add_stmt(a);
         }

         // False part
         {
            vhdl_abstract_assign_stmt *a = assign_for(atype, lhs2, rhs2);
            if (after)
               a->set_after(translate_time_expr(i_delay));
            vhdif->get_else_container()->add_stmt(a);
         }

         container->add_stmt(vhdif);
         return;
      }

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
      int lval_no = 0;
      for (it = lvals.begin(); it != lvals.end(); ++it, ++lval_no) {
         vhdl_var_ref *tmp_rhs = tmp_decl->make_ref();

         int lval_width = (*it)->get_type()->get_width();
         // A part of the target that ivl ignores (const_index_undefined):
         // its bits of the value are skipped, the rest are stored
         if (lval_store_ignored(ivl_stmt_lval(stmt, lval_no))) {
            width_so_far += ivl_lval_width(ivl_stmt_lval(stmt, lval_no));
            continue;
         }
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

         // Deposit or assign, as for a single target
         vhdl_decl::assign_type_t atype = assign_type;
         if (atype == vhdl_decl::ASSIGN_NONBLOCK && after == NULL
             && deposits_signal(proc, (*it)->get_name(), emul_blocking,
                                ivl_lval_sig(ivl_stmt_lval(stmt, lval_no)))) {
            atype = vhdl_decl::ASSIGN_BLOCK;
            proc->mark_deposited((*it)->get_name());
         }

         if (!check_valid_assignment(atype, proc, stmt))
            return;

         vhdl_abstract_assign_stmt *a = assign_for(atype, *it, tmp_rhs);
         if (after)
            a->set_after(after);

         container->add_stmt(a);

         width_so_far += lval_width;

         if (atype == vhdl_decl::ASSIGN_NONBLOCK && emul_blocking)
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

   // An intra-assignment event control (`x <= @(posedge c) v',
   // `x <= repeat (n) @(posedge c) v'): the value is taken now and stored
   // when the event comes, the n-th time for repeat (n), while the process
   // goes on. A VHDL signal assignment has a time delay (`after'), not an
   // event one; the translation dropped the event control and stored at once
   // (ivtest nb_ec_*: silently wrong values, or a run that never ended).
   if (ivl_stmt_nevent(stmt) > 0) {
      error("%s:%d: no VHDL translation for an intra-assignment event control "
            "on a nonblocking assignment (`x <= @(...) v', "
            "`x <= repeat (n) @(...) v')",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
      return 1;
   }

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

   // ($random(seed) draws and advances its seed ahead of the statement:
   // emit_seeded_random_pre)
   make_assignment(proc, container, stmt, emulate_blocking, assign_type);

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
   proc->left_time_zero();   // what follows runs after the delay

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

// Verilog's time-zero order (R6T-02): see vhdl_target.h. On unless
// SV2VHDL_TC08=0.
bool time_zero_order_enabled()
{
   static int on = -1;
   if (on < 0) {
      const char *e = getenv("SV2VHDL_TC08");
      on = (e == NULL || atoi(e) != 0);
   }
   return on != 0 && get_sv2vhdl_mode();
}

// The NBA wake-shadow close (draw_wait) is on unless SV2VHDL_NBA_SHADOW=0.
static bool nba_shadow_enabled()
{
   static int on = -1;
   if (on < 0) {
      const char *e = getenv("SV2VHDL_NBA_SHADOW");
      on = (e == NULL || atoi(e) != 0);
   }
   return on != 0 && get_sv2vhdl_mode();
}

// One trigger of an edge-triggered process watched across the NBA shadow:
// the scalar logic3d signal, its snapshot variable, and the edge kind (-1
// fall, +1 rise).  Empty `sig' when the trigger is not eligible.
struct shadow_arm_t {
   std::string sig, snap;
   int kind;
};

static shadow_arm_t shadow_arm_for(vhdl_process *proc, ivl_nexus_t nex,
                                   int kind)
{
   shadow_arm_t arm = { "", "", kind };
   vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), nex);
   if (ref != NULL && ref->get_type() != NULL && ref->get_slice() == NULL
       && ref->get_type()->get_name() == VHDL_TYPE_LOGIC3D) {
      arm.sig = ref->get_name();
      arm.snap = "v_icg2en_snap_" + arm.sig;
      while (proc->get_scope()->have_declared(arm.snap))
         arm.snap += "_";
   }
   return arm;
}

// The "missed edge" term of an arm: the trigger now sits on the edge's
// far side while its snapshot (taken before the NBA wait) did not.
static vhdl_expr *shadow_missed_edge(const shadow_arm_t &arm)
{
   const char *fn = (arm.kind < 0) ? "is_zero" : "is_one";
   vhdl_fcall *now_f = new vhdl_fcall(fn, vhdl_type::boolean());
   now_f->add_expr(new vhdl_var_ref(arm.sig.c_str(), vhdl_type::logic3d()));
   vhdl_fcall *was_f = new vhdl_fcall(fn, vhdl_type::boolean());
   was_f->add_expr(new vhdl_var_ref(arm.snap.c_str(), vhdl_type::logic3d()));
   return new vhdl_binop_expr(
      now_f, VHDL_BINOP_AND,
      new vhdl_unaryop_expr(VHDL_UNARYOP_NOT, was_f, vhdl_type::boolean()),
      vhdl_type::boolean());
}

// Declare an arm's snapshot and hand it to nba_defer_commits.  The initial
// value keeps the missed-edge term false until the first snapshot.
static void shadow_register(vhdl_process *proc, const shadow_arm_t &arm)
{
   vhdl_var_decl *sd = new vhdl_var_decl(arm.snap, vhdl_type::logic3d());
   sd->set_initial(new vhdl_var_ref(arm.kind < 0 ? "L3D_0" : "L3D_1",
                                    vhdl_type::logic3d()));
   proc->get_scope()->add_decl(sd);
   proc->add_icg2en_shadow(arm.sig, arm.snap, arm.kind);
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
   bool clock_rising = false;
   ivl_nexus_t the_clock_net = *clock_net.begin();
   for (int i = 0; i < nevents; i++) {
      ivl_event_t event = ivl_stmt_events(stmt, i);

      const unsigned npos = ivl_event_npos(event);
      for (unsigned j = 0; j < npos; j++) {
         if (ivl_event_pos(event, j) == the_clock_net) {
            edge = new vhdl_fcall("rising_edge", vhdl_type::boolean());
            clock_rising = true;
         }
      }

      const unsigned nneg = ivl_event_nneg(event);
      for (unsigned j = 0; j < nneg; j++)
         if (ivl_event_neg(event, j) == the_clock_net) {
            edge = new vhdl_fcall("falling_edge", vhdl_type::boolean());
            clock_rising = false;
         }
   }
   assert(edge);

   // ICG->enable rewrite for the async-reset template: the reset elsif
   // structure is untouched, only the clock term moves to the root
   // clock + latched-enable guard (see icg2en_pos_term)
   vhdl_expr *edge_test = NULL;
   std::string icg_sens;
   bool icg_rewrote = false;
   if (clock_rising) {
      edge_test = icg2en_pos_term(proc, the_clock_net, &icg_sens);
      if (edge_test != NULL)
         icg_rewrote = true;
   }
   if (edge_test == NULL) {
      edge->add_expr(nexus_to_var_ref(proc->get_scope(), the_clock_net));
      edge_test = edge;
   }
   else
      delete edge;

   // NBA wake-shadow close for this template too (draw_wait has it for the
   // generic shape): once nba_defer_commits gives the process its `wait
   // for 0 ns' epilogue, it is on no pending list while it sits there, so
   // a reset or clock edge settling later in the same instant (a derived
   // reset; a testbench reset at a clock edge) would be lost for good.
   // Snapshot both arms before that wait; when one moved, the process loops
   // back instead of re-arming: the level reset test then sees an asserted
   // reset, and the clock arm gets a missed-edge term.  An ICG2EN-rewritten
   // clock term is left alone.
   shadow_arm_t rst_arm = { "", "", 0 }, clk_arm = { "", "", 0 };
   if (nba_shadow_enabled()) {
      ivl_nexus_t rst_nex = *test_nexuses.begin();
      int rst_kind = 0;
      for (int i = 0; i < nevents; i++) {
         ivl_event_t event = ivl_stmt_events(stmt, i);
         for (unsigned j = 0; j < ivl_event_npos(event); j++)
            if (ivl_event_pos(event, j) == rst_nex)
               rst_kind = +1;
         for (unsigned j = 0; j < ivl_event_nneg(event); j++)
            if (ivl_event_neg(event, j) == rst_nex)
               rst_kind = -1;
      }
      if (rst_kind != 0)
         rst_arm = shadow_arm_for(proc, rst_nex, rst_kind);
      if (!icg_rewrote) {
         clk_arm = shadow_arm_for(proc, the_clock_net, clock_rising ? +1 : -1);
         if (!clk_arm.sig.empty() && clk_arm.snap == rst_arm.snap)
            clk_arm.snap += "_";
         if (!clk_arm.sig.empty())
            edge_test = new vhdl_binop_expr(edge_test, VHDL_BINOP_OR,
                                            shadow_missed_edge(clk_arm),
                                            vhdl_type::boolean());
      }
   }

   // Draw the clocked branch
   // For an asynchronous reset we just want this around the else branch,
   stmt_container *else_container = body->add_elsif(edge_test);

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

   if (!rst_arm.sig.empty())
      shadow_register(proc, rst_arm);
   if (!clk_arm.sig.empty())
      shadow_register(proc, clk_arm);

   // Add all the edge triggered signals to the sensitivity list (the
   // rewritten gated clock pends on the root instead)
   for (set<ivl_nexus_t>::const_iterator it = edge_triggered.begin();
        it != edge_triggered.end(); ++it) {
      if (icg_rewrote && *it == the_clock_net) {
         proc->add_sensitivity(icg_sens);
         continue;
      }
      // Get the signal that represents this nexus in this scope
      vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), *it);

      proc->add_sensitivity(ref->get_name());

      // Don't need the reference any more
      delete ref;
   }

   proc->set_edge_triggered();

   // Don't bother with the default draw_wait
   return true;
}


/*
 * SV2VHDL_ICG2EN: integrated-clock-gate -> clock-enable rewrite at
 * translation (interp twin of the gsm GSM_ICG2EN pass).  A gated clock
 *   assign gclk = clk & en_ff;            // AND
 *   always @(clk, en) if (!clk) en_ff = en;   // transparent-LOW latch
 * freezes en_ff across the clk-high phase, so for a posedge consumer
 *   always @(posedge gclk) BODY
 * the value of en_ff AT the root posedge equals the pre-edge value of
 * its input cone and
 *   always @(posedge clk) if (en_ff) BODY
 * is delta-race-free (the latch does nothing at clk high).  Consumers
 * then pend on the ROOT clock: one fastclk table / fused block covers
 * the design.  Transparent-HIGH latches are NOT equivalent (decline).
 * NEGEDGE consumers race the latch reopening in the same delta ->
 * decline (poisons the net).  The gated net itself is kept: value
 * readers and declined nets see it unchanged.  Per-net all-or-nothing:
 * one non-rewritable edge consumer poisons the net so a rewritten
 * (delta-earlier) flop can never feed a declined (delta-later) one on
 * the SAME net; cross-net skew against declined nets is accepted and
 * arbitrated by the differential gates.  Nested ICGs chase to the root
 * and conjoin each level's latched enable.
 */

struct icg_info_t {
   bool matched = false;
   bool poisoned = false;
   ivl_nexus_t root = NULL;                    // root clock after chasing
   ivl_nexus_t ck1 = NULL;                     // DIRECT clock input of the
                                               // gate driving this net —
                                               // level-1 rewrite target
                                               // (visible at the site by
                                               // construction; the chased
                                               // root need not be)
   std::vector<ivl_scope_t> en_scopes;         // ICG scope per level
   std::vector<ivl_signal_t> en_sigs;          // latched enable per level
   std::vector<char> en_const_one;             // per level: enable is
                                               // constant-1 (free-running
                                               // gate) — guard trivial,
                                               // no ports
   std::vector<std::vector<ivl_nexus_t> > en_input_sets;
                                               // per level: the nexuses
                                               // feeding the latch data
                                               // (E/TE after one OR
                                               // flatten) — these span
                                               // the hierarchy, so a
                                               // SITE can wire them as
                                               // synthetic guard ports
};

static std::map<ivl_nexus_t, icg_info_t> g_icg_cache;
static std::map<ivl_signal_t, std::pair<ivl_process_t, int> > g_sig_assigns;
static bool g_icg_scanned = false;

static bool icg2en_enabled()
{
   static int en = -1;
   if (en < 0) {
      const char *e = getenv("SV2VHDL_ICG2EN");
      en = (e != NULL && *e == '1') ? 1 : 0;
   }
   return en != 0;
}

// Debug sink: "1" -> stderr; any other value -> append to that path
// (the sv2ghdl driver redirects the backend's stderr to /dev/null)
static FILE *icg2en_debug_fp()
{
   static FILE *fp = NULL;
   static int init = 0;
   if (!init) {
      init = 1;
      const char *e = getenv("SV2VHDL_ICG2EN_DEBUG");
      if (e != NULL)
         fp = (e[0] == '1' && e[1] == '\0') ? stderr : fopen(e, "a");
   }
   return fp;
}

static bool icg2en_debug()
{
   return icg2en_debug_fp() != NULL;
}

// Recursive lval collector for the assign map
static void icg_collect_lvals(ivl_statement_t stmt, ivl_process_t proc)
{
   if (stmt == NULL)
      return;
   switch (ivl_statement_type(stmt)) {
   case IVL_ST_ASSIGN:
   case IVL_ST_ASSIGN_NB:
      for (unsigned i = 0; i < ivl_stmt_lvals(stmt); i++) {
         ivl_signal_t sig = ivl_lval_sig(ivl_stmt_lval(stmt, i));
         if (sig == NULL)
            continue;
         std::pair<ivl_process_t, int> &e = g_sig_assigns[sig];
         if (e.first != proc)
            e.second++;         // count DISTINCT assigning processes
         e.first = proc;
      }
      break;
   case IVL_ST_BLOCK:
   case IVL_ST_FORK:
      for (unsigned i = 0; i < ivl_stmt_block_count(stmt); i++)
         icg_collect_lvals(ivl_stmt_block_stmt(stmt, i), proc);
      break;
   case IVL_ST_CONDIT:
      icg_collect_lvals(ivl_stmt_cond_true(stmt), proc);
      icg_collect_lvals(ivl_stmt_cond_false(stmt), proc);
      break;
   case IVL_ST_CASE:
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
      for (unsigned i = 0; i < ivl_stmt_case_count(stmt); i++)
         icg_collect_lvals(ivl_stmt_case_stmt(stmt, i), proc);
      break;
   case IVL_ST_WAIT:
   case IVL_ST_DELAY:
   case IVL_ST_DELAYX:
   case IVL_ST_WHILE:
   case IVL_ST_FOREVER:
   case IVL_ST_REPEAT:
      icg_collect_lvals(ivl_stmt_sub_stmt(stmt), proc);
      break;
   default:
      break;
   }
}

extern "C" int icg_scan_assigns_cb(ivl_process_t proc, void *)
{
   icg_collect_lvals(ivl_process_stmt(proc), proc);
   return 0;
}

// Unwrap single-statement begin/end blocks
static ivl_statement_t icg_unwrap(ivl_statement_t stmt)
{
   while (stmt != NULL && ivl_statement_type(stmt) == IVL_ST_BLOCK
          && ivl_stmt_block_count(stmt) == 1)
      stmt = ivl_stmt_block_stmt(stmt, 0);
   return stmt;
}

// Is `s` the output of a transparent-LOW latch on ck_nexus?
//   always @(ck, ...) if (!ck) s = <expr>;   (blocking or NBA, no else)
static bool icg_match_latch(ivl_signal_t s, ivl_nexus_t ck_nexus)
{
   std::map<ivl_signal_t, std::pair<ivl_process_t, int> >::iterator it =
      g_sig_assigns.find(s);
   if (it == g_sig_assigns.end() || it->second.second != 1)
      return false;
   ivl_process_t proc = it->second.first;
   if (ivl_process_type(proc) != IVL_PR_ALWAYS)
      return false;
   ivl_statement_t w = ivl_process_stmt(proc);
   if (w == NULL || ivl_statement_type(w) != IVL_ST_WAIT)
      return false;
   bool ck_in_any = false;
   for (unsigned i = 0; i < (unsigned)ivl_stmt_nevent(w); i++) {
      ivl_event_t ev = ivl_stmt_events(w, i);
      if (ivl_event_npos(ev) > 0 || ivl_event_nneg(ev) > 0)
         return false;                      // edge-sensitive: not a latch
      for (unsigned j = 0; j < ivl_event_nany(ev); j++)
         if (ivl_event_any(ev, j) == ck_nexus)
            ck_in_any = true;
   }
   if (!ck_in_any)
      return false;
   ivl_statement_t c = icg_unwrap(ivl_stmt_sub_stmt(w));
   if (c == NULL || ivl_statement_type(c) != IVL_ST_CONDIT)
      return false;
   if (ivl_stmt_cond_false(c) != NULL)
      return false;
   ivl_expr_t cond = ivl_stmt_cond_expr(c);
   if (cond == NULL || ivl_expr_type(cond) != IVL_EX_UNARY
       || ivl_expr_opcode(cond) != '!')
      return false;
   ivl_expr_t ck = ivl_expr_oper1(cond);
   if (ck == NULL || ivl_expr_type(ck) != IVL_EX_SIGNAL
       || ivl_signal_nex(ivl_expr_signal(ck), 0) != ck_nexus)
      return false;
   ivl_statement_t a = icg_unwrap(ivl_stmt_cond_true(c));
   if (a == NULL || (ivl_statement_type(a) != IVL_ST_ASSIGN
                     && ivl_statement_type(a) != IVL_ST_ASSIGN_NB))
      return false;
   if (ivl_stmt_lvals(a) != 1 || ivl_lval_sig(ivl_stmt_lval(a, 0)) != s)
      return false;
   return true;
}

// Enable-source nexuses at the GATE CHAIN-TOP boundary.  Walk up from
// the matched latch\'s scope while each scope drives the gated net
// through one of its OUTPUT ports (the rvclkhdr/rvoclkhdr plumbing
// chain); the topmost such instance\'s width-1 INPUT ports — minus the
// clock input — are the enables, wired by construction in the scope
// where the gate chain is instantiated, so every SITE at that level
// can name them (and pass-through levels chain them by port).  This is
// per-instance (the chain belongs to one gate instance) and avoids
// tunnelling through hierarchy from the latch RHS, which surfaced
// nets in scopes a class-representative site cannot see.
// A constant-1 enable input marks the level free-running (guard
// trivially true: no enable term, no ports); constant-0 inputs drop.
static bool icg_scope_output_on(ivl_scope_t sc, ivl_nexus_t gnex)
{
   int n = ivl_scope_sigs(sc);
   for (int i = 0; i < n; i++) {
      ivl_signal_t sg = ivl_scope_sig(sc, i);
      if (ivl_signal_port(sg) == IVL_SIP_OUTPUT
          && ivl_signal_width(sg) == 1
          && ivl_signal_nex(sg, 0) == gnex)
         return true;
   }
   return false;
}

static int icg_nexus_const_bit(ivl_nexus_t nx)
{
   for (unsigned i = 0; i < ivl_nexus_ptrs(nx); i++) {
      ivl_net_const_t con = ivl_nexus_ptr_con(ivl_nexus_ptr(nx, i));
      if (con != NULL) {
         const char *bits = ivl_const_bits(con);
         if (bits != NULL && bits[0] == '1') return 1;
         return 0;
      }
   }
   return -1;
}

// A clock-plumbing wrapper has exactly ONE output port (l1clk/Q);
// anything with more outputs is real logic that happens to export a
// gated clock (e.g. a unit driving active_clk to siblings) and must
// terminate the walk — its inputs are NOT enables.
static bool icg_single_output_module(ivl_scope_t sc)
{
   int nout = 0;
   int n = ivl_scope_sigs(sc);
   for (int i = 0; i < n; i++) {
      ivl_signal_t sg = ivl_scope_sig(sc, i);
      if (ivl_signal_port(sg) == IVL_SIP_OUTPUT)
         nout++;
   }
   return nout == 1;
}

static std::vector<ivl_nexus_t> icg_gate_level_enables(
   ivl_nexus_t gnex, ivl_scope_t latch_scope, ivl_nexus_t ck_nex,
   bool *const_one)
{
   std::vector<ivl_nexus_t> out;
   *const_one = false;                 // retained for API stability
   // chain top: last single-output ancestor whose output drives the
   // gated net (the header wrapper chain)
   ivl_scope_t top = latch_scope;
   for (ivl_scope_t sc = latch_scope; sc != NULL;
        sc = ivl_scope_parent(sc)) {
      if (ivl_scope_type(sc) != IVL_SCT_MODULE)
         continue;
      if (icg_scope_output_on(sc, gnex)
          && icg_single_output_module(sc))
         top = sc;
      else if (sc != latch_scope)
         break;
   }
   // ALL non-clock width-1 inputs, in port order — constants stay in
   // the list so the synthetic port count and numbering are identical
   // for every instance of the class (sites wire literals for consts)
   int n = ivl_scope_sigs(top);
   for (int i = 0; i < n; i++) {
      ivl_signal_t sg = ivl_scope_sig(top, i);
      if (ivl_signal_port(sg) != IVL_SIP_INPUT
          || ivl_signal_width(sg) != 1)
         continue;
      ivl_nexus_t nx = ivl_signal_nex(sg, 0);
      if (nx == ck_nex)
         continue;                     // the clock input
      out.push_back(nx);
   }
   if (out.size() > 4 && icg2en_debug())
      fprintf(icg2en_debug_fp(), "icg2en: WIDE chain-top %s (%zu enables)\n",
              ivl_scope_name(top), out.size());
   return out;
}

static icg_info_t &icg_classify(ivl_nexus_t gnex, int depth);

static icg_info_t &icg_classify(ivl_nexus_t gnex, int depth)
{
   std::map<ivl_nexus_t, icg_info_t>::iterator hit = g_icg_cache.find(gnex);
   if (hit != g_icg_cache.end())
      return hit->second;
   icg_info_t &info = g_icg_cache[gnex];    // inserted unmatched = cycle guard
   if (depth > 4)
      return info;

   // Exactly one driver and it is a 1-bit two-input AND
   ivl_net_logic_t gate = NULL;
   for (unsigned i = 0; i < ivl_nexus_ptrs(gnex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(gnex, i);
      ivl_net_logic_t log = ivl_nexus_ptr_log(p);
      if (log != NULL && ivl_logic_pin(log, 0) == gnex) {
         if (gate != NULL)
            return info;                    // multiple gate drivers
         gate = log;
      }
      ivl_lpm_t lpm = ivl_nexus_ptr_lpm(p);
      if (lpm != NULL && ivl_lpm_q(lpm) == gnex)
         return info;
      if (ivl_nexus_ptr_con(p) != NULL)
         return info;
      if (ivl_nexus_ptr_switch(p) != NULL)
         return info;
   }
   if (gate == NULL || ivl_logic_type(gate) != IVL_LO_AND
       || ivl_logic_pins(gate) != 3 || ivl_logic_width(gate) != 1)
      return info;

   ivl_scope_t gscope = ivl_logic_scope(gate);
   ivl_nexus_t in[2] = { ivl_logic_pin(gate, 1), ivl_logic_pin(gate, 2) };

   for (int orient = 0; orient < 2 && !info.matched; orient++) {
      ivl_nexus_t en_nex = in[orient], ck_nex = in[1 - orient];
      for (unsigned i = 0; i < ivl_nexus_ptrs(en_nex); i++) {
         ivl_signal_t sig = ivl_nexus_ptr_sig(ivl_nexus_ptr(en_nex, i));
         if (sig == NULL || ivl_signal_scope(sig) != gscope)
            continue;
         if (icg_match_latch(sig, ck_nex)) {
            icg_info_t &inner = icg_classify(ck_nex, depth + 1);
            info.matched = true;
            info.ck1 = ck_nex;
            if (inner.matched) {
               info.root = inner.root;
               info.en_scopes = inner.en_scopes;
               info.en_sigs = inner.en_sigs;
               info.en_input_sets = inner.en_input_sets;
               info.en_const_one = inner.en_const_one;
            }
            else
               info.root = ck_nex;
            info.en_scopes.push_back(gscope);
            info.en_sigs.push_back(sig);
            {
               bool c1 = false;
               info.en_input_sets.push_back(
                  icg_gate_level_enables(gnex, gscope, ck_nex, &c1));
               info.en_const_one.push_back(c1 ? 1 : 0);
            }
            break;
         }
      }
   }
   if (icg2en_debug() && info.matched)
      fprintf(icg2en_debug_fp(), "icg2en: matched gate in %s (%zu enable level(s))\n",
              ivl_scope_name(gscope), info.en_sigs.size());
   return info;
}

// Poison pre-pass: any edge consumer of a matched gated net that the
// rewrite cannot carry (negedge, multi-event, mixed lists) disqualifies
// the WHOLE net.  Visibility/path checks happen at rewrite time and
// poison there (first consumer draws before any rewrite commits... the
// rewrite is per-process, so a late visibility failure would split the
// net; instead visibility is ALSO checked here, conservatively, per
// consumer module scope).
// A consumer is rewritable when exactly ONE matched gated net appears,
// as a single posedge, and every other edge in the wait rides an
// UNMATCHED net (the async-reset form: posedge gclk or negedge rst_l).
// Anything else — negedge on a gated net, several gated clocks, a
// gated net inside an any-edge list used as an edge — poisons every
// matched net the consumer touches (per-net all-or-nothing).
extern "C" int icg_poison_cb(ivl_process_t proc, void *)
{
   ivl_statement_t w = ivl_process_stmt(proc);
   if (w == NULL || ivl_statement_type(w) != IVL_ST_WAIT)
      return 0;
   const int nevents = ivl_stmt_nevent(w);

   std::vector<icg_info_t*> touched;
   int gated_pos = 0;
   bool bad = false;
   for (int i = 0; i < nevents; i++) {
      ivl_event_t ev = ivl_stmt_events(w, i);
      for (unsigned j = 0; j < ivl_event_nneg(ev); j++) {
         icg_info_t &inf = icg_classify(ivl_event_neg(ev, j), 0);
         if (inf.matched) { touched.push_back(&inf); bad = true; }
      }
      for (unsigned j = 0; j < ivl_event_npos(ev); j++) {
         icg_info_t &inf = icg_classify(ivl_event_pos(ev, j), 0);
         if (inf.matched) { touched.push_back(&inf); gated_pos++; }
      }
   }
   if (touched.empty())
      return 0;
   if (gated_pos > 1)
      bad = true;
   if (bad) {
      for (size_t k = 0; k < touched.size(); k++)
         touched[k]->poisoned = true;
      if (icg2en_debug())
         fprintf(icg2en_debug_fp(), "icg2en: net poisoned (unrewritable "
                 "consumer in %s)\n",
                 ivl_scope_name(ivl_process_scope(proc)));
   }
   return 0;
}

static void icg_scan_once()
{
   if (g_icg_scanned)
      return;
   g_icg_scanned = true;
   ivl_design_process(get_vhdl_design(), icg_scan_assigns_cb, NULL);
   ivl_design_process(get_vhdl_design(), icg_poison_cb, NULL);
}

// Relative instance path from the consumer module scope down to the ICG
// scope, dotted; empty when not a strict descendant
static std::string icg_rel_path(ivl_scope_t module, ivl_scope_t icg_scope)
{
   std::vector<std::string> chain;
   for (ivl_scope_t s = icg_scope; s != NULL; s = ivl_scope_parent(s)) {
      if (s == module) {
         std::string path;
         for (std::vector<std::string>::reverse_iterator it = chain.rbegin();
              it != chain.rend(); ++it) {
            if (!path.empty())
               path += ".";
            path += *it;
         }
         return path;
      }
      chain.push_back(ivl_scope_basename(s));
   }
   return "";
}

// Build the replacement posedge term for a matched, unpoisoned gated
// nexus: rising_edge(root_clk) and is_one(<<en>>)...; NULL = decline
static bool icg_nexus_in_scope(ivl_nexus_t nx, ivl_scope_t sc);
static vhdl_expr *icg2en_latched_ref(vhdl_arch *arch, vhdl_expr *ck_ref,
                                     vhdl_expr *en_ref,
                                     const std::string &key);

// Synthetic guard-port name for enable k of gated clock port `pbase`.
static std::string icg2en_port_name(const std::string &pbase, size_t k)
{
   char buf[16];
   snprintf(buf, sizeof(buf), "_e%zu", k);
   return "icg2en_" + pbase + buf;
}

static vhdl_expr *icg2en_pos_term(vhdl_process *proc, ivl_nexus_t gnex,
                                  std::string *sens_name)
{
   if (!icg2en_enabled() || !get_sv2vhdl_mode())
      return NULL;
   icg_scan_once();
   icg_info_t &info = icg_classify(gnex, 0);
   if (icg2en_debug())
      fprintf(icg2en_debug_fp(), "icg2en: pos_term m=%d p=%d\n",
              info.matched, info.poisoned);
   if (!info.matched || info.poisoned)
      return NULL;
   bool en_const1 = false;
   if (info.en_input_sets.empty()
       || info.en_input_sets.back().empty()) {
      info.poisoned = true;      // enable shape not port-wireable
      if (icg2en_debug())
         fprintf(icg2en_debug_fp(),
                 "icg2en: net poisoned (enable inputs unresolvable)\n");
      return NULL;
   }
   static const std::vector<ivl_nexus_t> icg_no_ens;
   const std::vector<ivl_nexus_t> &ens =
      en_const1 ? icg_no_ens : info.en_input_sets.back();

   // (the active scope may be a named block, task or function inside it)
   ivl_scope_t module = get_active_scope();
   while (module != NULL && (ivl_scope_type(module) == IVL_SCT_GENERATE
                             || ivl_scope_type(module) == IVL_SCT_BEGIN
                             || ivl_scope_type(module) == IVL_SCT_FORK
                             || ivl_scope_type(module) == IVL_SCT_TASK
                             || ivl_scope_type(module) == IVL_SCT_FUNCTION))
      module = ivl_scope_parent(module);

   // PORT MODE: the gated net arrives through this module\'s own clock
   // port; the split-entity signature covers it.  The SITE repoints the
   // clock actual at the root AND wires the enables into synthetic
   // guard ports (see icg2en_map_enables) -- no external names, which
   // the runtime mishandles at scale (campaign #76 round 13).
   {
      std::vector<std::string> up_paths;
      bool pm = (module != NULL)
         && icg2en_port_mode(module, gnex, &up_paths, false);
      if (pm) {
         // clock port basename (the port whose nexus is gnex)
         std::string pbase;
         int nsigs = ivl_scope_sigs(module);
         for (int i = 0; i < nsigs; i++) {
            ivl_signal_t psig = ivl_scope_sig(module, i);
            if (ivl_signal_port(psig) == IVL_SIP_INPUT
                && ivl_signal_width(psig) == 1
                && ivl_signal_nex(psig, 0) == gnex) {
               pbase = ivl_signal_basename(psig);
               break;
            }
         }
         if (pbase.empty()) {
            error("icg2en: signature-covered port not found in %s",
                  ivl_scope_name(module));
            return NULL;
         }
         vhdl_entity *ent = find_entity(module);
         if (ent == NULL) {
            error("icg2en: no entity for %s", ivl_scope_name(module));
            return NULL;
         }
         // Consistency with icg2en_map_enables: synthetic ports exist
         // on a class IFF the enables are NOT nameable inside it.  A
         // module that can name them all reads them directly.
         bool all_inside = true;
         for (size_t k = 0; k < ens.size(); k++)
            if (!icg_nexus_in_scope(ens[k], module))
               all_inside = false;
         vhdl_var_ref *port_ref =
            nexus_to_var_ref(proc->get_scope(), gnex);
         vhdl_fcall *edge =
            new vhdl_fcall("rising_edge", vhdl_type::boolean());
         edge->add_expr(port_ref);
         vhdl_binop_expr *conj =
            new vhdl_binop_expr(VHDL_BINOP_AND, vhdl_type::boolean());
         conj->add_expr(edge);
         vhdl_binop_expr *engroup =
            new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
         for (size_t k = 0; k < ens.size(); k++) {
            vhdl_fcall *is1 = new vhdl_fcall("is_one",
                                             vhdl_type::boolean());
            if (all_inside) {
               // read through a replica latch (header-latch init/
               // sampling semantics — see icg2en_latched_ref)
               vhdl_var_ref *er =
                  nexus_to_var_ref(proc->get_scope(), ens[k]);
               is1->add_expr(icg2en_latched_ref(ent->get_arch(),
                  nexus_to_var_ref(proc->get_scope(), info.ck1),
                  er, er->get_name()));
            }
            else {
               std::string pname = icg2en_port_name(pbase, k);
               if (!ent->get_scope()->have_declared(pname))
                  ent->add_port(new vhdl_port_decl(pname.c_str(),
                     vhdl_type::logic3d(), VHDL_PORT_IN));
               is1->add_expr(new vhdl_var_ref(pname.c_str(),
                                              vhdl_type::logic3d()));
            }
            engroup->add_expr(is1);
         }
         if (!ens.empty())
            conj->add_expr(engroup);
         *sens_name = port_ref->get_name();
         if (icg2en_debug())
            fprintf(icg2en_debug_fp(), "icg2en: PORT-MODE rewrite in %s "
                    "(%zu enable port(s)%s)\n", ivl_scope_name(module),
                    ens.size(), en_const1 ? ", free-running" : "");
         return conj;
      }
   }

   // DESCENDANT MODE: the gate lives inside this module\'s subtree; the
   // enable-source nets must be visible right here (they feed the gate
   // chain\'s top instance, wired in some scope at-or-below this one).
   if (!nexus_visible_in_scope(proc->get_scope(), info.root)) {
      info.poisoned = true;      // all-or-nothing: keep the net whole
      if (icg2en_debug())
         fprintf(icg2en_debug_fp(),
                 "icg2en: net poisoned (root clock not visible)\n");
      return NULL;
   }
   for (size_t k = 0; k < ens.size(); k++) {
      if (!nexus_visible_in_scope(proc->get_scope(), ens[k])) {
         info.poisoned = true;
         if (icg2en_debug())
            fprintf(icg2en_debug_fp(),
                    "icg2en: net poisoned (enable input not visible)\n");
         return NULL;
      }
   }

   vhdl_var_ref *clk_ref = nexus_to_var_ref(proc->get_scope(), info.root);
   vhdl_fcall *edge = new vhdl_fcall("rising_edge", vhdl_type::boolean());
   edge->add_expr(clk_ref);

   vhdl_binop_expr *conj =
      new vhdl_binop_expr(VHDL_BINOP_AND, vhdl_type::boolean());
   conj->add_expr(edge);
   vhdl_binop_expr *engroup =
      new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
   {
      vhdl_entity *cent = module != NULL ? find_entity(module) : NULL;
      for (size_t k = 0; k < ens.size(); k++) {
         vhdl_fcall *is1 = new vhdl_fcall("is_one", vhdl_type::boolean());
         vhdl_var_ref *er = nexus_to_var_ref(proc->get_scope(), ens[k]);
         if (cent != NULL && nexus_visible_in_scope(proc->get_scope(),
                                                    info.ck1))
            is1->add_expr(icg2en_latched_ref(cent->get_arch(),
               nexus_to_var_ref(proc->get_scope(), info.ck1),
               er, er->get_name()));
         else
            is1->add_expr(er);
         engroup->add_expr(is1);
      }
   }
   if (!ens.empty())
      conj->add_expr(engroup);

   *sens_name = clk_ref->get_name();
   if (icg2en_debug())
      fprintf(icg2en_debug_fp(), "icg2en: rewrote posedge consumer in %s "
              "-> root + %zu enable input(s)%s\n",
              module ? ivl_scope_name(module) : "?", ens.size(),
              en_const1 ? ", free-running" : "");
   return conj;
}


// ---- ICG2EN entity-splitting signature --------------------------------
// A module whose clock PORT is fed by a matched ICG must be emitted as
// a SEPARATE specialization from raw-clocked instances of the same
// module: the dedup key (same_scope_type_name) is extended with this
// signature.  The signature records, per gated input port, the
// upward-relative path from the module to each level's latched enable
// ("<port>@<up>^<inst.path.en>;..."), so every member of one gated
// class shares the parent shape by construction and one emitted entity
// (guard = upward external name) serves them all.  Sites of the gated
// class pass the ROOT clock as the port actual (see map_signal), so
// the entity's rising_edge(port) IS the root edge.
static std::map<ivl_scope_t, std::string> g_icg_sig_cache;

bool icg2en_key_enabled()
{
   return icg2en_enabled();
}

// Emitted instance labels, recorded by draw_hierarchy when each
// vhdl_comp_inst label is finalized (labels transform basenames: []
// stripping, underscore rules, entity-name "_inst" suffixing,
// collision avoidance).  Guard paths must use THESE; the dedup-key
// signature keeps raw basenames (computed before labels exist).
// Keyed by (parent entity class, instance basename): a label is a
// property of the parent CLASS's architecture — one drawn
// representative labels the hop for every instance of the class,
// including chains under non-representative parents.
static std::map<std::pair<const vhdl_entity*, std::string>,
                std::string> g_icg_labels;

void icg2en_note_label(ivl_scope_t scope, const std::string &label)
{
   ivl_scope_t parent = ivl_scope_parent(scope);
   while (parent != NULL && ivl_scope_type(parent) != IVL_SCT_MODULE)
      parent = ivl_scope_parent(parent);
   if (parent == NULL)
      return;
   const vhdl_entity *pent = find_entity(parent);
   if (pent == NULL)
      return;
   g_icg_labels[std::make_pair(pent,
      std::string(ivl_scope_basename(scope)))] = label;
}

static std::string icg_label_for(ivl_scope_t scope)
{
   ivl_scope_t parent = ivl_scope_parent(scope);
   while (parent != NULL && ivl_scope_type(parent) != IVL_SCT_MODULE)
      parent = ivl_scope_parent(parent);
   if (parent == NULL)
      return "";
   const vhdl_entity *pent = find_entity(parent);
   if (pent == NULL)
      return "";
   std::map<std::pair<const vhdl_entity*, std::string>,
            std::string>::iterator it =
      g_icg_labels.find(std::make_pair(pent,
         std::string(ivl_scope_basename(scope))));
   return it == g_icg_labels.end() ? "" : it->second;
}

// Upward-relative path from consumer module `from` to signal `sig`
// under `sig_scope`, counted in EMITTED-hierarchy hops: generate
// scopes flatten into their enclosing module's architecture, so only
// MODULE scopes count as caret levels or path components.  "N^a.b.s".
// use_labels: true = emitted labels (guard emission; empty when a hop
// has no recorded label), false = raw basenames (dedup-key signature).
static std::string icg_uprel_path2(ivl_scope_t from, ivl_scope_t sig_scope,
                                   ivl_signal_t sig, bool use_labels)
{
   // Module-scope ancestors of `from`: anchor candidates
   std::vector<ivl_scope_t> anchors;
   anchors.push_back(from);
   for (ivl_scope_t sc = ivl_scope_parent(from);
        sc != NULL && anchors.size() <= 4; sc = ivl_scope_parent(sc))
      if (ivl_scope_type(sc) == IVL_SCT_MODULE)
         anchors.push_back(sc);

   for (size_t up = 0; up < anchors.size(); up++) {
      ivl_scope_t anc = anchors[up];
      // Downward chain of MODULE instance scopes from sig_scope to anc
      std::vector<ivl_scope_t> chain;
      bool ok = false;
      for (ivl_scope_t sc = sig_scope; sc != NULL;
           sc = ivl_scope_parent(sc)) {
         if (sc == anc) { ok = true; break; }
         if (ivl_scope_type(sc) == IVL_SCT_MODULE)
            chain.push_back(sc);
      }
      if (!ok)
         continue;
      std::string path;
      char buf[16];
      snprintf(buf, sizeof(buf), "%zu^", up);
      path = buf;
      for (std::vector<ivl_scope_t>::reverse_iterator it = chain.rbegin();
           it != chain.rend(); ++it) {
         std::string comp;
         if (use_labels) {
            comp = icg_label_for(*it);
            if (comp.empty())
               return "";        // label unknown: cannot emit safely
         }
         else
            comp = ivl_scope_basename(*it);
         path += comp + ".";
      }
      path += ivl_signal_basename(sig);
      return path;
   }
   return "";
}

static std::string icg_uprel_path(ivl_scope_t from, ivl_scope_t sig_scope,
                                  ivl_signal_t sig)
{
   return icg_uprel_path2(from, sig_scope, sig, false);
}

// Static wireability of the synthetic guard ports for a consumer of
// gated net `gnex`: at the consumer's parent, each enable must be a
// constant (wired as a literal), nameable, or chainable through a
// parent that itself carries the gated net on an input port (the
// pass-through recursion map_enables performs).  Consumers that fail
// (e.g. the lsu_clkdomain topology where the gate lives in a SIBLING
// and its enable is an internal comb there) are declined up front so
// signature/sites/guard stay consistent.
static bool icg2en_can_wire(ivl_scope_t consumer, ivl_nexus_t gnex,
                            const std::vector<ivl_nexus_t> &ens,
                            int depth)
{
   if (depth > 4)
      return false;
   ivl_scope_t p = ivl_scope_parent(consumer);
   while (p != NULL && ivl_scope_type(p) != IVL_SCT_MODULE)
      p = ivl_scope_parent(p);
   if (p == NULL)
      return false;
   bool need_chain = false;
   for (size_t k = 0; k < ens.size(); k++) {
      if (icg_nexus_const_bit(ens[k]) >= 0)
         continue;
      if (icg_nexus_in_scope(ens[k], p))
         continue;
      need_chain = true;
   }
   if (!need_chain)
      return true;
   // chain: the parent must carry the gated net on an input port
   int n = ivl_scope_sigs(p);
   for (int i = 0; i < n; i++) {
      ivl_signal_t sg = ivl_scope_sig(p, i);
      if (ivl_signal_port(sg) == IVL_SIP_INPUT
          && ivl_signal_width(sg) == 1
          && ivl_signal_nex(sg, 0) == gnex)
         return icg2en_can_wire(p, gnex, ens, depth + 1);
   }
   return false;
}

std::string icg2en_scope_signature(ivl_scope_t scope)
{
   if (!icg2en_enabled() || !get_sv2vhdl_mode())
      return "";
   if (ivl_scope_type(scope) != IVL_SCT_MODULE)
      return "";
   std::map<ivl_scope_t, std::string>::iterator hit =
      g_icg_sig_cache.find(scope);
   if (hit != g_icg_sig_cache.end())
      return hit->second;
   icg_scan_once();

   std::string sig_str;
   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t psig = ivl_scope_sig(scope, i);
      if (ivl_signal_port(psig) != IVL_SIP_INPUT)
         continue;
      if (ivl_signal_width(psig) != 1)
         continue;
      icg_info_t &info = icg_classify(ivl_signal_nex(psig, 0), 0);
      if (!info.matched || info.poisoned)
         continue;
      // SV2VHDL_ICG2EN_FILTER: comma-separated substrings — only ICGs
      // whose instance path matches rewrite (bisection tool)
      {
         static const char *flt = getenv("SV2VHDL_ICG2EN_FILTER");
         if (flt != NULL && *flt != '\0') {
            const char *ipath = ivl_scope_name(info.en_scopes.back());
            bool hit = false;
            std::string f(flt);
            size_t pos = 0;
            while (pos != std::string::npos) {
               size_t c = f.find(',', pos);
               std::string tok = f.substr(pos,
                  c == std::string::npos ? std::string::npos : c - pos);
               if (!tok.empty() && strstr(ipath, tok.c_str()) != NULL)
                  hit = true;
               pos = (c == std::string::npos) ? std::string::npos : c + 1;
            }
            if (!hit)
               continue;
         }
      }
      // CLOCK INFRASTRUCTURE EXCLUSION: if this port feeds an ICG
      // *inside* this module's subtree (its AND takes the net as an
      // input), the module is clock-gating plumbing (rvclkhdr-class)
      // and must keep the REAL clock — repointing its actual would
      // feed the outer clock into the inner gate's CP and bypass a
      // gating level.  Only leaf consumers rewrite.
      {
         ivl_nexus_t pnex = ivl_signal_nex(psig, 0);
         bool feeds_icg = false;
         for (unsigned pi = 0; pi < ivl_nexus_ptrs(pnex) && !feeds_icg;
              pi++) {
            ivl_net_logic_t log =
               ivl_nexus_ptr_log(ivl_nexus_ptr(pnex, pi));
            if (log == NULL || ivl_logic_pin(log, 0) == pnex)
               continue;               // not a gate, or the net's driver
            // gate INPUT on this net: inside this module's subtree?
            bool inside = false;
            for (ivl_scope_t sc = ivl_logic_scope(log); sc != NULL;
                 sc = ivl_scope_parent(sc))
               if (sc == scope) { inside = true; break; }
            if (inside && icg_classify(ivl_logic_pin(log, 0), 0).matched)
               feeds_icg = true;
         }
         if (feeds_icg)
            continue;
      }
      // VALUE-READER exclusion: if anything inside this module's
      // subtree reads the gated net as DATA — a gate or LPM input tap
      // (memory write strobes, latch-style clocking, clock muxes) —
      // repointing the port actual would silently ungate that logic.
      // Only modules whose consumers are exclusively rewritable
      // posedge processes (checked by the poison pre-pass) or
      // pass-through instantiations are eligible.
      {
         ivl_nexus_t pnex = ivl_signal_nex(psig, 0);
         bool value_read = false;
         for (unsigned pi = 0; pi < ivl_nexus_ptrs(pnex) && !value_read;
              pi++) {
            ivl_nexus_ptr_t np = ivl_nexus_ptr(pnex, pi);
            ivl_net_logic_t log = ivl_nexus_ptr_log(np);
            ivl_lpm_t lpm = ivl_nexus_ptr_lpm(np);
            ivl_scope_t rsc = NULL;
            if (log != NULL && ivl_logic_pin(log, 0) != pnex)
               rsc = ivl_logic_scope(log);
            else if (lpm != NULL && ivl_lpm_q(lpm) != pnex)
               rsc = ivl_lpm_scope(lpm);
            if (rsc == NULL)
               continue;
            for (ivl_scope_t sc = rsc; sc != NULL;
                 sc = ivl_scope_parent(sc))
               if (sc == scope) { value_read = true; break; }
         }
         if (value_read) {
            if (icg2en_debug())
               fprintf(icg2en_debug_fp(),
                       "icg2en: %s.%s declined (value reader in subtree)\n",
                       ivl_scope_name(scope), ivl_signal_basename(psig));
            continue;
         }
      }
      // Guard-port wireability: every enable must be wireable at the
      // consumer's site (literal / nameable / port-chained) — decline
      // the port otherwise so signature, sites and guard agree
      if (info.en_input_sets.empty() || info.en_input_sets.back().empty()
          || !icg2en_can_wire(scope, ivl_signal_nex(psig, 0),
                              info.en_input_sets.back(), 0)) {
         if (icg2en_debug())
            fprintf(icg2en_debug_fp(),
                    "icg2en: %s.%s declined (guards not wireable)\n",
                    ivl_scope_name(scope), ivl_signal_basename(psig));
         continue;
      }
      // LEVEL-1 semantics: the rewrite moves the consumer one gating
      // level up (to the gate's direct clock input, visible at the
      // site) guarded by that level's latched enable; nested chains
      // consolidate one level per net
      std::string p = icg_uprel_path(scope, info.en_scopes.back(),
                                     info.en_sigs.back());
      if (p.empty())
         continue;      // unreachable enable: this port stays plain
      sig_str += std::string(ivl_signal_basename(psig)) + "@" + p + ";";
   }
   g_icg_sig_cache[scope] = sig_str;
   if (!sig_str.empty() && icg2en_debug())
      fprintf(icg2en_debug_fp(), "icg2en: scope %s signature %s\n",
              ivl_scope_name(scope), sig_str.c_str());
   return sig_str;
}

// Does `scope`'s signature cover the port whose nexus is `gnex`?
// Returns the enable paths ("N^a.b.en") for term emission.
bool icg2en_port_mode(ivl_scope_t scope, ivl_nexus_t gnex,
                      std::vector<std::string> *paths, bool use_labels)
{
   const std::string sig = icg2en_scope_signature(scope);
   if (sig.empty())
      return false;
   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t psig = ivl_scope_sig(scope, i);
      if (ivl_signal_port(psig) != IVL_SIP_INPUT
          || ivl_signal_width(psig) != 1
          || ivl_signal_nex(psig, 0) != gnex)
         continue;
      // The port must be COVERED by the signature: the signature loop
      // applies every per-port exclusion (FILTER, clock-infrastructure,
      // guard wireability) — a port it skipped is NOT in port mode even
      // when sibling ports are.
      const std::string marker = std::string(ivl_signal_basename(psig)) + "@";
      if (sig.find(marker) == std::string::npos)
         return false;
      icg_info_t &info = icg_classify(gnex, 0);
      if (!info.matched || info.poisoned)
         return false;
      std::string p = icg_uprel_path2(scope, info.en_scopes.back(),
                                      info.en_sigs.back(), use_labels);
      if (p.empty())
         return false;
      paths->push_back(p);
      return true;
   }
   return false;
}

// Site-side: should this child port actual be re-pointed at the root
// clock?  True when the CHILD's signature covers the port.
bool icg2en_site_root(ivl_signal_t child_port, ivl_nexus_t *root_out)
{
   if (!icg2en_enabled() || !get_sv2vhdl_mode())
      return false;
   if (ivl_signal_port(child_port) != IVL_SIP_INPUT
       || ivl_signal_width(child_port) != 1)
      return false;
   ivl_scope_t cscope = ivl_signal_scope(child_port);
   std::vector<std::string> paths;
   if (!icg2en_port_mode(cscope, ivl_signal_nex(child_port, 0), &paths,
                         false))
      return false;
   icg_info_t &info = icg_classify(ivl_signal_nex(child_port, 0), 0);
   *root_out = info.ck1;
   if (icg2en_debug())
      fprintf(icg2en_debug_fp(), "icg2en: site repoints %s.%s to root\n",
              ivl_scope_name(cscope), ivl_signal_basename(child_port));
   return true;
}

// Replica enable latch.  The clockhdr latch initialises to L3D_X
// (value 0) and holds that until its FIRST CP-low sample, so the
// original gated clock's first rise comes one edge later than the raw
// enable suggests; guards must reproduce that or rewritten flops fire
// one edge early during the reset/X window (campaign #76 round 18:
// X-din captures at roots 10/20ns that the original never made).
// Wire every guard input through a per-(arch, enable) replica:
//   process (ck, en)  if is_zero(ck) then lat <= en;  end if;
// with the same L3D_X initial — semantics identical to the header
// latch at every sampling instant, nameable at the wiring site, and
// shared by every consumer wired at this arch.  `en_ref` may be a
// literal (constant enables latch too — the first half-cycle of X
// matters even for .en(1'b1) free-running gates).
static vhdl_expr *icg2en_latched_ref(vhdl_arch *arch, vhdl_expr *ck_expr,
                                     vhdl_expr *en_ref,
                                     const std::string &key)
{
   vhdl_var_ref *ck_ref = dynamic_cast<vhdl_var_ref*>(ck_expr);
   if (ck_ref == NULL)
      return en_ref;               // cannot build the latch: raw fallback
   vhdl_scope *ascope = arch->get_scope();
   std::string lname = "icg2en_lat_" + key;
   if (!ascope->have_declared(lname)) {
      vhdl_signal_decl *decl =
         new vhdl_signal_decl(lname.c_str(), vhdl_type::logic3d());
      decl->set_initial(new vhdl_var_ref("L3D_X", vhdl_type::logic3d()));
      ascope->add_decl(decl);

      vhdl_process *proc = new vhdl_process();
      proc->add_sensitivity(ck_ref->get_name());
      {
         vhdl_var_ref *er = dynamic_cast<vhdl_var_ref*>(en_ref);
         if (er != NULL && er->get_name().find("L3D_") != 0)
            proc->add_sensitivity(er->get_name());
      }
      vhdl_fcall *is0 = new vhdl_fcall("is_zero", vhdl_type::boolean());
      is0->add_expr(ck_ref);
      vhdl_if_stmt *iflow = new vhdl_if_stmt(is0);
      iflow->get_then_container()->add_stmt(new vhdl_nbassign_stmt(
         new vhdl_var_ref(lname.c_str(), vhdl_type::logic3d()), en_ref));
      proc->get_container()->add_stmt(iflow);
      arch->add_stmt(proc);
   }
   return new vhdl_var_ref(lname.c_str(), vhdl_type::logic3d());
}


// Site-side: wire the synthetic guard ports of a signature-covered
// child.  For every gated input clock port of `child` the child entity
// declares icg2en_<port>_e<k> in-ports (see icg2en_pos_term PORT MODE);
// the site must associate them with the enable-source nets.  The
// enable nexuses span the hierarchy: at the ICG-adjacent site they are
// visible directly; at a pass-through site the PARENT module itself
// carries the same gated nexus on one of its own clock ports, so the
// parent gains the same synthetic ports (ensured here) and the child\'s
// ports chain to them name-to-name.
// Does a signal on `nx` live DIRECTLY in module scope `sc`?  (= the
// nexus is nameable inside that module's architecture)
static bool icg_nexus_in_scope(ivl_nexus_t nx, ivl_scope_t sc)
{
   for (unsigned i = 0; i < ivl_nexus_ptrs(nx); i++) {
      ivl_signal_t sg = ivl_nexus_ptr_sig(ivl_nexus_ptr(nx, i));
      if (sg != NULL && ivl_signal_scope(sg) == sc)
         return true;
   }
   return false;
}

// Entity-side: declare the synthetic guard ports for every signature-
// covered gated clock port of `scope` at ENTITY CREATION.  The port
// list must follow from the signature ALONE: a covered module with no
// rewritable posedge process (e.g. a memory whose gated clock only
// feeds latch-style logic) never reaches icg2en_pos_term, but sites
// still associate the ports.  Unused in-ports are harmless.
void icg2en_add_entity_ports(ivl_scope_t scope, vhdl_entity *ent)
{
   if (!icg2en_enabled() || !get_sv2vhdl_mode())
      return;
   icg_scan_once();
   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t psig = ivl_scope_sig(scope, i);
      if (ivl_signal_port(psig) != IVL_SIP_INPUT
          || ivl_signal_width(psig) != 1)
         continue;
      ivl_nexus_t gnex = ivl_signal_nex(psig, 0);
      std::vector<std::string> raw_paths;
      if (!icg2en_port_mode(scope, gnex, &raw_paths, false))
         continue;
      icg_info_t &info = icg_classify(gnex, 0);
      if (info.en_input_sets.empty() || info.en_input_sets.back().empty())
         continue;
      // Consistency rule (see icg2en_map_enables): nameable-inside
      // classes carry no ports
      const std::vector<ivl_nexus_t> &ens = info.en_input_sets.back();
      bool all_inside = true;
      for (size_t k = 0; k < ens.size(); k++)
         if (!icg_nexus_in_scope(ens[k], scope))
            all_inside = false;
      if (all_inside)
         continue;
      std::string cbase = ivl_signal_basename(psig);
      for (size_t k = 0; k < ens.size(); k++) {
         std::string pname = icg2en_port_name(cbase, k);
         if (!ent->get_scope()->have_declared(pname))
            ent->add_port(new vhdl_port_decl(pname.c_str(),
               vhdl_type::logic3d(), VHDL_PORT_IN));
      }
   }
}

void icg2en_map_enables(ivl_scope_t child, const vhdl_entity *parent_c,
                        vhdl_comp_inst *inst)
{
   if (!icg2en_enabled() || !get_sv2vhdl_mode())
      return;
   icg_scan_once();
   vhdl_entity *parent = const_cast<vhdl_entity*>(parent_c);
   int nsigs = ivl_scope_sigs(child);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t psig = ivl_scope_sig(child, i);
      if (ivl_signal_port(psig) != IVL_SIP_INPUT
          || ivl_signal_width(psig) != 1)
         continue;
      ivl_nexus_t gnex = ivl_signal_nex(psig, 0);
      std::vector<std::string> raw_paths;
      if (!icg2en_port_mode(child, gnex, &raw_paths, false))
         continue;
      icg_info_t &info = icg_classify(gnex, 0);
      if (info.en_input_sets.empty() || info.en_input_sets.back().empty())
         continue;
      const std::vector<ivl_nexus_t> &ens = info.en_input_sets.back();
      // If the child module can name every enable itself, its inner
      // sites (or its own guarded process, in descendant mode) wire
      // them internally and its entity has NO synthetic ports — the
      // outer site must not associate any.  Consistency rule: ports
      // exist on a class IFF the enables are NOT nameable inside it.
      {
         bool all_inside = true;
         for (size_t k = 0; k < ens.size(); k++)
            if (!icg_nexus_in_scope(ens[k], child))
               all_inside = false;
         if (all_inside)
            continue;
      }
      std::string cbase = ivl_signal_basename(psig);
      vhdl_scope *ascope = parent->get_arch()->get_scope();
      for (size_t k = 0; k < ens.size(); k++) {
         std::string cname = icg2en_port_name(cbase, k);
         // local clock ref for the replica latch (the gate's direct
         // clock input, nameable at every chain level by construction)
         seen_nexus(info.ck1);
         // constant enable at this instance (e.g. free_cg .en(1'b1)):
         // still LATCHED — the first-half-cycle X of the header latch
         // is semantically significant even for constants
         {
            int cb = icg_nexus_const_bit(ens[k]);
            if (cb >= 0 && nexus_visible_in_scope(ascope, info.ck1)) {
               char kb[64];
               snprintf(kb, sizeof(kb), "c%d_%s_%zu", cb, cbase.c_str(), k);
               inst->map_port(cname, icg2en_latched_ref(
                  parent->get_arch(),
                  nexus_to_var_ref(ascope, info.ck1),
                  new vhdl_var_ref(cb ? "L3D_1" : "L3D_0",
                                   vhdl_type::logic3d()),
                  kb));
               continue;
            }
         }
         seen_nexus(ens[k]);     // populate nexus private before the
                                 // visibility test (map_signal parity)
         if (nexus_visible_in_scope(ascope, ens[k])
             && nexus_visible_in_scope(ascope, info.ck1)) {
            vhdl_var_ref *er = nexus_to_var_ref(ascope, ens[k]);
            inst->map_port(cname, icg2en_latched_ref(
               parent->get_arch(),
               nexus_to_var_ref(ascope, info.ck1), er,
               er->get_name()));
            continue;
         }
         // pass-through: does the parent module carry the same gated
         // nexus on one of its own input ports?
         ivl_scope_t pmod = ivl_scope_parent(child);
         while (pmod != NULL && ivl_scope_type(pmod) != IVL_SCT_MODULE)
            pmod = ivl_scope_parent(pmod);
         std::string pbase;
         if (pmod != NULL) {
            int pn = ivl_scope_sigs(pmod);
            for (int pi = 0; pi < pn; pi++) {
               ivl_signal_t ps = ivl_scope_sig(pmod, pi);
               if (ivl_signal_port(ps) == IVL_SIP_INPUT
                   && ivl_signal_width(ps) == 1
                   && ivl_signal_nex(ps, 0) == gnex) {
                  pbase = ivl_signal_basename(ps);
                  break;
               }
            }
         }
         if (!pbase.empty()) {
            std::string pname = icg2en_port_name(pbase, k);
            if (!parent->get_scope()->have_declared(pname))
               parent->add_port(new vhdl_port_decl(pname.c_str(),
                  vhdl_type::logic3d(), VHDL_PORT_IN));
            inst->map_port(cname,
               new vhdl_var_ref(pname.c_str(), vhdl_type::logic3d()));
            continue;
         }
         if (icg2en_debug()) {
            fprintf(icg2en_debug_fp(),
                    "icg2en: WIRE-FAIL %s of %s in %s; enable signals:\n",
                    cname.c_str(), ivl_scope_name(child),
                    parent->get_name().c_str());
            for (unsigned d = 0; d < ivl_nexus_ptrs(ens[k]); d++) {
               ivl_signal_t sg = ivl_nexus_ptr_sig(ivl_nexus_ptr(ens[k], d));
               if (sg != NULL)
                  fprintf(icg2en_debug_fp(), "   %s.%s\n",
                          ivl_scope_name(ivl_signal_scope(sg)),
                          ivl_signal_basename(sg));
            }
         }
         error("icg2en: cannot wire guard port %s of %s at site in %s "
               "(enable not visible, no pass-through port)",
               cname.c_str(), ivl_scope_name(child),
               parent->get_name().c_str());
      }
   }
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

   // What follows an event control runs once the event comes
   proc->left_time_zero();

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

      // Inside a block (`@(a) stmt' after other statements, in an initial
      // or a task) the event comes first: the statement runs once `a'
      // changes, not before it (the end-of-process wait is only the
      // top-level loop form).  Draw the body aside, place the wait first.
      // So does a top-level `always @(a or b)' under Verilog's time-zero
      // order (time_zero_order_enabled): vvp starts it waiting, it does
      // not run at time zero (always_comb and always_latch, which have a
      // time-zero trigger, still do).
      bool wait_first = !is_top_level;
      if (is_top_level && time_zero_order_enabled()
          && !ivl_stmt_needs_t0_trigger(stmt)) {
         int nany_total = 0;
         for (int i = 0; i < nevents; i++)
            nany_total += ivl_event_nany(ivl_stmt_events(stmt, i));
         wait_first = nany_total > 0;
      }
      stmt_container body_aside;
      draw_stmt(proc, wait_first ? &body_aside : container,
                ivl_stmt_sub_stmt(stmt), true);

      vhdl_wait_stmt *wait = NULL;
      if (proc->contains_wait_stmt() || wait_first)
         wait = new vhdl_wait_stmt(VHDL_WAIT_ON);
      // An event control inside the body suspends the process: it can have
      // no sensitivity list, and its leading event control becomes a wait
      if (wait_first)
         proc->added_wait_stmt();

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
      if (wait_first)
         container->move_stmts_from(&body_aside);
   }
   else {
      // Build a test expression to represent the edge event
      // If this process contains no `wait' statements and this
      // is the top-level container, then we
      // wrap it in an `if' statement with this test and add the
      // edge triggered signals to the sensitivity, otherwise
      // build a `wait until' statement at the top of the process
      if (is_top_level)
         proc->set_edge_triggered();
      vhdl_binop_expr *test =
         new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());

      // ICG->enable delta alignment (SV2VHDL_ICG2EN): a rewritten flop
      // pends on the ROOT clock and would otherwise wake one delta
      // EARLIER than the original gated flop (whose gclk came through
      // the header's AND gate, +1 delta).  Downstream TRANSPARENT
      // LATCHES (other clock headers' en_ff) sample combinational
      // enables mid-instant, so the skew latches into persistent state
      // divergence (VeeR: 4 IFU header en_ffs at the reset instant).
      // Restore the original timing by inserting `wait for 0 ns` at
      // the top of the fire branch -- but ONLY when entered via the
      // clock arm: async (reset) arms woke the original directly with
      // no gate delay.  The async decision is cached in a variable at
      // the wake delta because 'event attributes are stale after the
      // wait.  Pending stays on the root net: the one-table
      // consolidation is preserved.
      vhdl_binop_expr *icg_async_test =
         new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
      bool icg_rewrote = false, icg_has_async = false;
      // (name, kind) of every async arm, for the wake-shadow close below.
      // kind: -1 fall, +1 rise, 0 any.  Only scalar logic3d arms are
      // eligible (vector compares can't be folded into edge terms).
      std::list<std::pair<std::string,int> > icg_async_sigs;

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
            {
               vhdl_var_ref *r2 = nexus_to_var_ref(proc->get_scope(), nexus);
               if (r2->get_type() != NULL && r2->get_type()->get_name() == VHDL_TYPE_LOGIC3D)
                  icg_async_sigs.push_back(std::make_pair(r2->get_name(), 0));
               r2->set_name(r2->get_name() + "'Event");
               icg_async_test->add_expr(r2);
               icg_has_async = true;
            }

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
            {
               vhdl_var_ref *r2 = nexus_to_var_ref(proc->get_scope(), nexus);
               if (r2->get_type() != NULL && r2->get_type()->get_name() == VHDL_TYPE_LOGIC3D)
                  icg_async_sigs.push_back(std::make_pair(r2->get_name(), -1));
               vhdl_fcall *d2 =
                  new vhdl_fcall("falling_edge", vhdl_type::boolean());
               d2->add_expr(r2);
               icg_async_test->add_expr(d2);
               icg_has_async = true;
            }

            if (!proc->contains_wait_stmt() && is_top_level)
               proc->add_sensitivity(ref->get_name());
         }

         int npos = ivl_event_npos(event);
         for (int j = 0; j < npos; j++) {
            ivl_nexus_t nexus = ivl_event_pos(event, j);

            // ICG->enable rewrite: substitute this posedge TERM with
            // rising_edge(root) and is_one(enable) — sound per-term in
            // any event list (fires exactly when the gated net would
            // rise); the poison pre-pass has already rejected shapes
            // with several gated clocks or a gated negedge
            {
               std::string sens;
               vhdl_expr *term = icg2en_pos_term(proc, nexus, &sens);
               if (term != NULL) {
                  test->add_expr(term);
                  icg_rewrote = true;
                  if (!proc->contains_wait_stmt() && is_top_level)
                     proc->add_sensitivity(sens);
                  continue;
               }
            }

            vhdl_var_ref *ref = nexus_to_var_ref(proc->get_scope(), nexus);
            vhdl_fcall *detect =
               new vhdl_fcall("rising_edge", vhdl_type::boolean());
            detect->add_expr(ref);

            test->add_expr(detect);
            {
               vhdl_var_ref *r2 = nexus_to_var_ref(proc->get_scope(), nexus);
               if (r2->get_type() != NULL && r2->get_type()->get_name() == VHDL_TYPE_LOGIC3D)
                  icg_async_sigs.push_back(std::make_pair(r2->get_name(), +1));
               vhdl_fcall *d2 =
                  new vhdl_fcall("rising_edge", vhdl_type::boolean());
               d2->add_expr(r2);
               icg_async_test->add_expr(d2);
               icg_has_async = true;
            }

            if (!proc->contains_wait_stmt() && is_top_level)
               proc->add_sensitivity(ref->get_name());
         }
      }

      // Wake-shadow close: while an edge-triggered process sits at the
      // NBA `wait for 0 ns` it is off every pending list, so an async
      // trigger (e.g. a derived reset like VeeR's core_rst_l AND gate)
      // committing in a later delta of the same instant is dropped
      // forever.  This is a TRANSLATION-WIDE latent hole, not an ICG2EN
      // one: any flop whose clock event coincides with the instant a
      // derived reset settles can miss the reset and silently hold X
      // (the pre-fix ICG2EN control build measurably dropped 30ns
      // resets that the closed build took).  ICG2EN merely widens the
      // exposure by re-pointing flops one delta earlier than their old
      // gated clocks.  Close the hole for EVERY clocked process with
      // async arms, using per-trigger snapshot variables: OR
      // value-compare "missed edge" terms into the fire test, and (in
      // nba_defer_commits) skip the trailing re-arm wait when a trigger
      // moved during the shadow so the process loops and handles it
      // immediately.  Residual: a full pulse entirely inside the shadow
      // stays value-invisible; vector async triggers are skipped.
      // Escape hatch: SV2VHDL_NBA_SHADOW=0 restores the old shape.
      static int nba_shadow = -1;
      if (nba_shadow < 0) {
         const char *e = getenv("SV2VHDL_NBA_SHADOW");
         nba_shadow = (e == NULL || atoi(e) != 0);
      }
      if (nba_shadow && get_sv2vhdl_mode() && !icg_async_sigs.empty()) {
         for (std::list<std::pair<std::string,int> >::const_iterator it =
                 icg_async_sigs.begin(); it != icg_async_sigs.end(); ++it) {
            // Mixed any+edge lists are exotic and a level-compare term
            // has no statically-safe initial: skip kind-0 arms
            if (it->second == 0)
               continue;

            std::string snap = "v_icg2en_snap_" + it->first;
            while (proc->get_scope()->have_declared(snap))
               snap += "_";
            vhdl_var_decl *sd =
               new vhdl_var_decl(snap, vhdl_type::logic3d());
            // Initial value keeps the missed-edge term PERMANENTLY false
            // if this process never receives the NBA epilogue (variable-
            // only writes leave it sensitivity-style with the snapshot
            // never assigned): fall-detect needs is_zero(snap) true,
            // rise-detect needs is_one(snap) true
            sd->set_initial(new vhdl_var_ref(
               it->second < 0 ? "L3D_0" : "L3D_1", vhdl_type::logic3d()));
            proc->get_scope()->add_decl(sd);

            vhdl_expr *term = NULL;
            {
               const char *fn = (it->second < 0) ? "is_zero" : "is_one";
               vhdl_fcall *now_f = new vhdl_fcall(fn, vhdl_type::boolean());
               now_f->add_expr(
                  new vhdl_var_ref(it->first.c_str(), vhdl_type::logic3d()));
               vhdl_fcall *was_f = new vhdl_fcall(fn, vhdl_type::boolean());
               was_f->add_expr(
                  new vhdl_var_ref(snap.c_str(), vhdl_type::logic3d()));
               term = new vhdl_binop_expr(
                  now_f, VHDL_BINOP_AND,
                  new vhdl_unaryop_expr(VHDL_UNARYOP_NOT, was_f,
                                        vhdl_type::boolean()),
                  vhdl_type::boolean());
            }
            test->add_expr(term);
            proc->add_icg2en_shadow(it->first, snap, it->second);
         }
      }

      // Build the delta-alignment prologue when any term was rewritten
      stmt_container icg_prologue;
      if (icg_rewrote) {
         // Alignment depth: number of deltas between the root clock
         // edge and the original gated-clock rise.  The header chain is
         // deeper than the AND gate alone: the port-map concatenation
         // (a => CP & en_ff) is an implicit process (+1) and output
         // -Readable shadows add another.  Default 2; override with
         // SV2VHDL_ICG2EN_DELTAS while calibrating.
         int ndelta = 0;   // REFUTED: STD_MX routes wait-for-0 to the INACTIVE region (not a delta) and the NBA shadow contract already neutralizes flop-read skew; a nonzero value also moves the din read past the pre-edge snapshot. Kept for experiments only.
         {
            const char *e = getenv("SV2VHDL_ICG2EN_DELTAS");
            if (e != NULL) ndelta = atoi(e);
         }
         if (icg_has_async) {
            const char *vn = "v_icg2en_async";
            proc->get_scope()->add_decl(
               new vhdl_var_decl(vn, vhdl_type::boolean()));
            icg_prologue.add_stmt(new vhdl_assign_stmt(
               new vhdl_var_ref(vn, vhdl_type::boolean()), icg_async_test));
            vhdl_if_stmt *align = new vhdl_if_stmt(
               new vhdl_unaryop_expr(VHDL_UNARYOP_NOT,
                  new vhdl_var_ref(vn, vhdl_type::boolean()),
                  vhdl_type::boolean()));
            for (int k = 0; k < ndelta; k++)
               align->get_then_container()->add_stmt(
                  new vhdl_wait_stmt(VHDL_WAIT_FOR0));
            icg_prologue.add_stmt(align);
         }
         else {
            for (int k = 0; k < ndelta; k++)
               icg_prologue.add_stmt(new vhdl_wait_stmt(VHDL_WAIT_FOR0));
         }
      }

      if (proc->contains_wait_stmt() || !is_top_level) {
         container->add_stmt(new vhdl_wait_stmt(VHDL_WAIT_UNTIL, test));
         if (!is_top_level)
            proc->added_wait_stmt();   // as for `wait on' above
         if (icg_rewrote)
            container->move_stmts_from(&icg_prologue);
         container->move_stmts_from(&tmp_container);
      }
      else {
         // Wrap the whole process body in an `if' statement to detect
         // the edge event
         vhdl_if_stmt *edge_detect = new vhdl_if_stmt(test);

         if (icg_rewrote)
            edge_detect->get_then_container()->move_stmts_from(&icg_prologue);

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

/*
 * Verilog case equality compares 4-state values, never strengths (IEEE
 * 1364 9.5; a strength is not part of an expression's value): a pull or
 * weak 1 -- a released pad's pull-up, a tri1 net, the weak drive an AMS
 * BIDIR A2D puts on an inout -- matches 1'b1, a weak unknown matches
 * 1'bx.  Map a scalar logic3d onto the strong codes the case items carry:
 * l3d_strengthen(l3d_weaken(x)) is L,0 -> 0; H,1 -> 1; Z -> Z; W,X,U -> X.
 */
static vhdl_expr *case_value_l3d(vhdl_expr *e)
{
   vhdl_fcall *weak = new vhdl_fcall("l3d_weaken", vhdl_type::logic3d());
   weak->add_expr(e);
   vhdl_fcall *strong = new vhdl_fcall("l3d_strengthen", vhdl_type::logic3d());
   strong->add_expr(weak);
   return strong;
}

static vhdl_var_ref *draw_case_test(vhdl_procedural *proc, stmt_container *container,
                                    ivl_statement_t stmt)
{
   vhdl_expr *test = translate_expr(ivl_stmt_cond_expr(stmt));
   if (NULL == test)
      return NULL;

   // The selector may read the target of a blocking assignment just made
   // (`r = pad; case (r)'): let it settle first, as draw_if does
   emit_wait_for_0(proc, container, stmt, test);

   // In sv2vhdl mode, extract the unsigned value field for case matching
   if (get_sv2vhdl_mode() && test->get_type()
       && test->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      int width = test->get_type()->get_width();
      vhdl_fcall *conv = new vhdl_fcall("l3d_to_unsigned",
                                         vhdl_type::nunsigned(width));
      conv->add_expr(test);
      test = conv;
   }
   // A scalar selector is compared without its strength (case_value_l3d);
   // casez/casex take it from here too (draw_casezx_l3d)
   else if (get_sv2vhdl_mode() && test->get_type()
            && test->get_type()->get_name() == VHDL_TYPE_LOGIC3D)
      test = case_value_l3d(test);

   // VHDL case expressions are required to be quite simple: variable
   // references or slices. So we may need to create a temporary
   // variable to hold the result of the expression evaluation
   if (typeid(*test) != typeid(vhdl_var_ref)) {
      // Find a unique name for the case expression variable.
      // Nested cases may have different widths or types (a scalar logic3d
      // and a 1-bit unsigned are both one wide), so they need separate
      // variables.
      const vhdl_type *test_type = new vhdl_type(*test->get_type());
      std::string tmp_name_str = "Verilog_Case_Ex";
      int suffix = 0;
      while (proc->get_scope()->have_declared(tmp_name_str)
             && (proc->get_scope()->get_decl(tmp_name_str)->get_type()->get_width()
                    != test_type->get_width()
                 || proc->get_scope()->get_decl(tmp_name_str)->get_type()->get_name()
                    != test_type->get_name())) {
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
         emit_wait_for_0(proc, container, stmt, when);
         when = when->cast(test->get_type());
         if (!when) return 1;
         // A scalar item is a value too (`case (1'b1) pad:'): drop its
         // strength as the selector's was (draw_case_test)
         if (get_sv2vhdl_mode() && test->get_type()
             && test->get_type()->get_name() == VHDL_TYPE_LOGIC3D)
            when = case_value_l3d(when);

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


// L3D_<bit> (0, 1, X or Z) in sv2vhdl mode
static vhdl_expr *l3d_const(char bit)
{
   return new vhdl_const_bit(bit);
}

static vhdl_expr *l3d_eq(vhdl_expr *a, vhdl_expr *b)
{
   return new vhdl_binop_expr(a, VHDL_BINOP_EQ, b, vhdl_type::boolean());
}

/*
 * casez/casex on a scalar selector (sv2vhdl mode).  The selector arrives
 * canonical from draw_case_test (0, 1, X or Z, strength dropped); each
 * 1-bit item matches when it equals the selector or either side is a
 * don't-care: Z (and ?) for casez, X and Z for casex (IEEE 1364 9.5.1).
 * Returns -1 when the statement is not of that shape (an item wider than
 * the selector): the caller reports it.
 */
static int draw_casezx_l3d(vhdl_procedural *proc, stmt_container *container,
                           ivl_statement_t stmt, bool is_last,
                           vhdl_var_ref *test)
{
   const bool is_casez = ivl_statement_type(stmt) == IVL_ST_CASEZ;
   const int nbranches = ivl_stmt_case_count(stmt);
   for (int i = 0; i < nbranches; i++) {
      ivl_expr_t net = ivl_stmt_case_expr(stmt, i);
      if (net && ivl_expr_width(net) != 1)
         return -1;
   }

   const string tname = test->get_name();
   vhdl_if_stmt *result = NULL;
   ivl_statement_t default_stmt = NULL;
   for (int i = 0; i < nbranches; i++) {
      ivl_expr_t net = ivl_stmt_case_expr(stmt, i);
      if (net == NULL) {
         default_stmt = ivl_stmt_case_stmt(stmt, i);
         continue;
      }
      vhdl_binop_expr *any =
         new vhdl_binop_expr(VHDL_BINOP_OR, vhdl_type::boolean());
      // the selector's own don't-cares
      any->add_expr(l3d_eq(new vhdl_var_ref(tname.c_str(), vhdl_type::logic3d()),
                           l3d_const('z')));
      if (!is_casez)
         any->add_expr(l3d_eq(new vhdl_var_ref(tname.c_str(), vhdl_type::logic3d()),
                              l3d_const('x')));
      if (ivl_expr_type(net) == IVL_EX_NUMBER) {
         const char bit = ivl_expr_bits(net)[0];
         if (bit == 'z' || bit == '?' || (!is_casez && bit == 'x'))
            any->add_expr(new vhdl_const_bool(true));
         else
            any->add_expr(l3d_eq(new vhdl_var_ref(tname.c_str(),
                                                  vhdl_type::logic3d()),
                                 l3d_const(bit)));
      }
      else {
         // A non-constant item: its value, strength dropped, may itself be
         // a don't-care.  (Translated once per use: trees are not shared.)
         vhdl_type l3(VHDL_TYPE_LOGIC3D);
         vhdl_expr *item[3];
         for (int k = 0; k < 3; k++) {
            item[k] = translate_expr(net);
            if (item[k] == NULL)
               return 1;
            emit_wait_for_0(proc, container, stmt, item[k]);
            item[k] = case_value_l3d(item[k]->cast(&l3));
         }
         any->add_expr(l3d_eq(item[0], l3d_const('z')));
         if (!is_casez)
            any->add_expr(l3d_eq(item[1], l3d_const('x')));
         any->add_expr(l3d_eq(new vhdl_var_ref(tname.c_str(),
                                               vhdl_type::logic3d()),
                              item[2]));
      }

      stmt_container *where;
      if (result == NULL) {
         result = new vhdl_if_stmt(any);
         where = result->get_then_container();
      }
      else
         where = result->add_elsif(any);
      draw_stmt(proc, where, ivl_stmt_case_stmt(stmt, i), is_last);
   }

   if (result == NULL) {
      // Only a default
      if (default_stmt)
         draw_stmt(proc, container, default_stmt, is_last);
      return 0;
   }
   if (default_stmt)
      draw_stmt(proc, result->get_else_container(), default_stmt, is_last);

   ostringstream ss;
   ss << "Generated from case" << (is_casez ? 'z' : 'x')
      << " statement at " << ivl_stmt_file(stmt) << ":" << ivl_stmt_lineno(stmt);
   result->set_comment(ss.str());
   container->add_stmt(result);
   return 0;
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

   if (get_sv2vhdl_mode() && test->get_type()
       && test->get_type()->get_name() == VHDL_TYPE_LOGIC3D) {
      int rc = draw_casezx_l3d(proc, container, stmt, is_last, test);
      if (rc >= 0)
         return rc;
      error("%s:%d: Sorry, a case%s statement with a 1-bit selector and "
            "wider labels cannot be translated to VHDL", ivl_stmt_file(stmt),
            ivl_stmt_lineno(stmt),
            ivl_statement_type(stmt) == IVL_ST_CASEZ ? "z" : "x");
      return 1;
   }

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

// A loop's body can run at any time: while it is drawn, a deposit is no
// blocking target (vhdl_procedural::at_time_zero)
namespace {
struct loop_body_t {
   vhdl_procedural *p;
   explicit loop_body_t(vhdl_procedural *pp) : p(pp) { p->enter_loop(); }
   ~loop_body_t() { p->leave_loop(); }
};
}

int draw_while(vhdl_procedural *proc, stmt_container *container,
               ivl_statement_t stmt, ivl_statement_t step=0)
{
   loop_body_t in_loop(proc);

   // A break or continue in the body (begin_loop_jumps): open the labels
   // for both drawings of the body below
   vhdl_labeled_loop_stmt *cont = NULL;
   vhdl_labeled_loop_stmt *brk =
      begin_loop_jumps(ivl_stmt_sub_stmt(stmt), cont);

   // Generate the body inside a temporary container before
   // generating the test
   // The reason for this is that some of the signals in the
   // test might be renamed while expanding the body (e.g. if
   // we need to generate an assignment to a constant signal)
   stmt_container tmp_container;
   int rc = draw_stmt(proc, &tmp_container, ivl_stmt_sub_stmt(stmt));
   if (rc != 0) {
      end_loop_jumps();
      return 1;
   }
   // When we are emitting a for as a while we need to add the step
   // (draw_stmt, not draw_assign: what the step's expression puts ahead of
   // it -- a $random(seed) draw -- belongs in the same container)
   if (step) {
      rc = draw_stmt(proc, &tmp_container, step);
      if (rc != 0) {
         end_loop_jumps();
         return rc;
      }
   }

   // The test runs before every pass, and so must what its expression puts
   // ahead of it (a $random(seed) draw): that goes to `pre'.  ($dist_* and
   // $value$plusargs there stay a located error: begin_conditional_eval)
   stmt_container pre;
   begin_conditional_eval();
   vhdl_expr *test = translate_loop_test(ivl_stmt_cond_expr(stmt), &pre);
   end_conditional_eval();
   if (NULL == test) {
      end_loop_jumps();
      return 1;
   }

   // The test must be a Boolean (and std_logic and (un)signed types
   // must be explicitly cast unlike in Verilog)
   vhdl_type boolean(VHDL_TYPE_BOOLEAN);
   test = test->cast(&boolean);

   if (!pre.empty())
      return draw_while_drawn_test(proc, container, stmt, step, test, &pre,
                                   brk, cont);

   emit_wait_for_0(proc, container, stmt, test);

   vhdl_while_stmt *loop = new vhdl_while_stmt(test);
   draw_stmt(proc, continue_body(cont, loop->get_container()),
             ivl_stmt_sub_stmt(stmt));
   close_continue(cont, loop->get_container());
   end_loop_jumps();

   // When we are emitting a for as a while we need to add the step
   if (step) {
      rc = draw_stmt(proc, loop->get_container(), step);
      if (rc != 0)
         return rc;
   }

   emit_wait_for_0(proc, loop->get_container(), stmt, test);

   container->add_stmt(wrap_break(brk, loop));
   return 0;
}

/*
 * SystemVerilog do <body> while (<cond>): the body runs before each test,
 *
 *    loop
 *       <body>
 *       if not <cond> then exit; end if;
 *    end loop;
 */
int draw_do_while(vhdl_procedural *proc, stmt_container *container,
                  ivl_statement_t stmt)
{
   loop_body_t in_loop(proc);

   vhdl_labeled_loop_stmt *cont = NULL;
   vhdl_labeled_loop_stmt *brk =
      begin_loop_jumps(ivl_stmt_sub_stmt(stmt), cont);

   vhdl_loop_stmt *loop = new vhdl_loop_stmt;
   int rc = draw_stmt(proc, continue_body(cont, loop->get_container()),
                      ivl_stmt_sub_stmt(stmt));
   close_continue(cont, loop->get_container());
   end_loop_jumps();
   if (rc != 0)
      return rc;

   begin_conditional_eval();   // the test runs after every pass
   vhdl_expr *test = translate_expr(ivl_stmt_cond_expr(stmt));
   end_conditional_eval();
   if (NULL == test)
      return 1;
   vhdl_type boolean(VHDL_TYPE_BOOLEAN);
   test = test->cast(&boolean);
   emit_wait_for_0(proc, loop->get_container(), stmt, test);

   vhdl_if_stmt *done = new vhdl_if_stmt(
      new vhdl_unaryop_expr(VHDL_UNARYOP_NOT, test, vhdl_type::boolean()));
   done->get_then_container()->add_stmt(new vhdl_exit_stmt());
   loop->get_container()->add_stmt(done);

   container->add_stmt(wrap_break(brk, loop));
   return 0;
}

/*
 * draw_while for a test whose expression put statements ahead of it, `pre'
 * (a $random(seed) draw): they run before every test,
 *
 *    loop
 *       <pre>
 *       if not <test> then exit; end if;
 *       <body> <step>
 *    end loop;
 *
 * with draw_while's break/continue targets (begin_loop_jumps: `brk' and
 * `cont', closed here by end_loop_jumps): the body goes in the continue
 * wrapper, the step after it, and the loop in the break wrapper.
 */
static int draw_while_drawn_test(vhdl_procedural *proc,
                                 stmt_container *container,
                                 ivl_statement_t stmt, ivl_statement_t step,
                                 vhdl_expr *test, stmt_container *pre,
                                 vhdl_labeled_loop_stmt *brk,
                                 vhdl_labeled_loop_stmt *cont)
{
   vhdl_loop_stmt *loop = new vhdl_loop_stmt;
   stmt_container *body = loop->get_container();
   body->move_stmts_from(pre);
   emit_wait_for_0(proc, body, stmt, test);
   vhdl_if_stmt *leave = new vhdl_if_stmt(
      new vhdl_unaryop_expr(VHDL_UNARYOP_NOT, test, vhdl_type::boolean()));
   leave->get_then_container()->add_stmt(new vhdl_exit_stmt());
   body->add_stmt(leave);
   int rc = draw_stmt(proc, continue_body(cont, body), ivl_stmt_sub_stmt(stmt));
   close_continue(cont, body);
   end_loop_jumps();
   if (rc == 0 && step)
      rc = draw_stmt(proc, body, step);
   if (rc != 0)
      return rc;
   container->add_stmt(wrap_break(brk, loop));
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
   loop_body_t in_loop(proc);

   vhdl_labeled_loop_stmt *cont = NULL;
   vhdl_labeled_loop_stmt *brk =
      begin_loop_jumps(ivl_stmt_sub_stmt(stmt), cont);

   vhdl_loop_stmt *loop = new vhdl_loop_stmt;
   container->add_stmt(wrap_break(brk, loop));

   draw_stmt(proc, continue_body(cont, loop->get_container()),
             ivl_stmt_sub_stmt(stmt));
   close_continue(cont, loop->get_container());
   end_loop_jumps();

   return 0;
}

int draw_repeat(vhdl_procedural *proc, stmt_container *container,
                ivl_statement_t stmt)
{
   ivl_expr_t cond = ivl_stmt_cond_expr(stmt);
   vhdl_expr *times = translate_expr(cond);
   if (NULL == times)
      return 1;

   // The count is read once, on entry: like the tests of if/while/case, it
   // must see a blocking assignment just made to a signal (`n = 3;
   // repeat (n)', or a task's input argument `repeat (k)')
   emit_wait_for_0(proc, container, stmt, times);

   vhdl_type integer(VHDL_TYPE_INTEGER);
   // A signed repeat count that is negative means zero iterations in Verilog;
   // an unsigned To_Integer would read -1 as ~2**31 and loop almost forever.
   // Reinterpret a signed logic3d count as signed so a negative count yields a
   // null `1 to N' range.
   if (get_sv2vhdl_mode() && ivl_expr_signed(cond) && times->get_type()
       && times->get_type()->get_name() == VHDL_TYPE_LOGIC3D_VECTOR) {
      vhdl_fcall *s = new vhdl_fcall("l3d_to_signed",
         vhdl_type::nsigned(times->get_type()->get_width()));
      s->add_expr(times);
      vhdl_fcall *ti = new vhdl_fcall("To_Integer", vhdl_type::integer());
      ti->add_expr(s);
      times = ti;
   }
   else
      times = times->cast(&integer);

   loop_body_t in_loop(proc);
   vhdl_labeled_loop_stmt *cont = NULL;
   vhdl_labeled_loop_stmt *brk =
      begin_loop_jumps(ivl_stmt_sub_stmt(stmt), cont);

   const char *it_name = "Verilog_Repeat";
   vhdl_for_stmt *loop =
      new vhdl_for_stmt(it_name, new vhdl_const_int(1), times);
   container->add_stmt(wrap_break(brk, loop));

   draw_stmt(proc, continue_body(cont, loop->get_container()),
             ivl_stmt_sub_stmt(stmt));
   close_continue(cont, loop->get_container());
   end_loop_jumps();

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

   // A SystemVerilog class task (a method) has no translation
   ivl_scope_t towner = ivl_scope_parent(tscope);
   if (towner != NULL && ivl_scope_type(towner) == IVL_SCT_CLASS) {
      error("unsupported construct (class) at %s:%d: %s() of SystemVerilog "
            "class %s has no VHDL translation", ivl_stmt_file(stmt),
            ivl_stmt_lineno(stmt), ivl_scope_basename(tscope),
            ivl_scope_tname(towner));
      return 1;
   }

   // TODO: adding some comments to the output would be helpful

   // A task is inlined into its caller: a task that calls itself, directly
   // or through another, would be inlined for ever (and an automatic one's
   // activations would share one copy of its variables)
   static std::set<ivl_scope_t> inlining;
   if (inlining.count(tscope)) {
      error("%s:%d: task %s calls itself (recursion): tasks are inlined in "
            "VHDL, so a recursive task call has no translation",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt), ivl_scope_name(tscope));
      return 1;
   }
   inlining.insert(tscope);

   // The inlined body runs in the task's scope: %m and the Scope line of
   // $error & co. name it (top.tk, as vvp does).
   ivl_scope_t prev_scope = get_active_scope();
   set_active_scope(tscope);

   // A `disable <task>' (or SV `return') inside the task leaves the inlined
   // body: it goes in a loop the disable exits (begin_disable_scope)
   vhdl_labeled_loop_stmt *dloop =
      begin_disable_scope(tscope, ivl_scope_def(tscope));

   // TODO: this completely ignores parameters!
   draw_stmt(proc, dloop ? dloop->get_container() : container,
             ivl_scope_def(tscope), false);

   if (dloop)
      end_disable_scope(container, dloop);
   set_active_scope(prev_scope);
   inlining.erase(tscope);
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

// Verilog force (and, approximately, procedural continuous assign): emit the
// VHDL-2008 force assignment. nvc's runtime gives the Verilog semantics: the
// forced value overrides all drivers and deposits; on release a net returns
// to its resolved driving value while a driverless reg retains the forced
// value. `assign r = v` is mapped to the same machinery -- the only divergence
// is Verilog's force-over-assign layering when both are active at once.
// A statically out-of-range ARRAY WORD lvalue: the Verilog write is a no-op.
static bool lval_word_statically_dead(vhdl_procedural *proc, vhdl_var_ref *lhs)
{
   if (!get_sv2vhdl_mode() || lhs == NULL || lhs->get_slice() == NULL)
      return false;
   vhdl_decl *decl = proc->get_scope()->get_decl(lhs->get_name());
   if (!decl || !decl->get_type()
       || decl->get_type()->get_name() != VHDL_TYPE_ARRAY)
      return false;
   vhdl_const_int *wb = dynamic_cast<vhdl_const_int*>(lhs->get_slice());
   if (!wb)
      return false;
   const int wlo = std::min(decl->get_type()->get_lsb(),
                            decl->get_type()->get_msb());
   const int whi = std::max(decl->get_type()->get_lsb(),
                            decl->get_type()->get_msb());
   return wb->get_value() < wlo || wb->get_value() > whi;
}

static int draw_force(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt)
{
   list<vhdl_var_ref*> lvals;
   if (!assignment_lvals(stmt, proc, lvals))
      return 1;
   if (lvals.size() != 1) {
      error("force with %zu lvalues not supported at %s:%d", lvals.size(),
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
      return 1;
   }
   vhdl_expr *rhs = translate_expr(ivl_stmt_rval(stmt));
   if (NULL == rhs)
      return 1;
   vhdl_var_ref *lhs = lvals.front();
   if (lval_word_statically_dead(proc, lhs))
      return 0;    // force to an out-of-range array word: lost
   note_forced_net(ivl_lval_sig(ivl_stmt_lval(stmt, 0)));
   rhs = rhs->cast(lhs->get_type());
   container->add_stmt(new vhdl_force_stmt(lhs, rhs));
   return 0;
}

static int draw_release(vhdl_procedural *proc, stmt_container *container,
                        ivl_statement_t stmt)
{
   list<vhdl_var_ref*> lvals;
   if (!assignment_lvals(stmt, proc, lvals))
      return 1;
   if (lvals.size() != 1) {
      error("release with %zu lvalues not supported at %s:%d", lvals.size(),
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
      return 1;
   }
   if (lval_word_statically_dead(proc, lvals.front()))
      return 0;    // release of an out-of-range array word: no-op
   note_forced_net(ivl_lval_sig(ivl_stmt_lval(stmt, 0)));
   container->add_stmt(new vhdl_release_stmt(lvals.front()));
   return 0;
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
/*
 * Pre-statement hook.  A few system functions have side effects that a VHDL
 * expression cannot express ($value$plusargs writes its second argument).
 * While a statement is being drawn, g_pre_container is the container it will
 * be added to; expression translation can append statements there, and they
 * land before the statement being built (whose condition/RHS is translated
 * before the statement itself is added).
 */
static vhdl_procedural *g_pre_proc = NULL;
static stmt_container *g_pre_container = NULL;
static ivl_statement_t g_pre_stmt = NULL;      // the statement being drawn
// True while a loop test is translated into a container of its own that
// runs before every test (translate_loop_test)
static bool g_pre_per_pass = false;

static int draw_stmt_inner(vhdl_procedural *proc, stmt_container *container,
                           ivl_statement_t stmt, bool is_last);

int draw_stmt(vhdl_procedural *proc, stmt_container *container,
              ivl_statement_t stmt, bool is_last)
{
   vhdl_procedural *save_proc = g_pre_proc;
   stmt_container *save_container = g_pre_container;
   ivl_statement_t save_stmt = g_pre_stmt;
   g_pre_proc = proc;
   g_pre_container = container;
   g_pre_stmt = stmt;
   int rc = draw_stmt_inner(proc, container, stmt, is_last);
   g_pre_proc = save_proc;
   g_pre_container = save_container;
   g_pre_stmt = save_stmt;
   return rc;
}

static vhdl_expr *translate_loop_test(ivl_expr_t cond, stmt_container *pre)
{
   stmt_container *save = g_pre_container;
   const bool save_per_pass = g_pre_per_pass;
   g_pre_container = pre;
   g_pre_per_pass = true;
   vhdl_expr *e = translate_expr(cond);
   g_pre_container = save;
   g_pre_per_pass = save_per_pass;
   return e;
}

/*
 * A comment line ahead of the statement being drawn: a `null;' carrying
 * it, for a note an expression makes (a system function replaced by a
 * constant, expr.cc).  False outside a procedural statement.
 */
bool emit_pre_comment(const std::string &text)
{
   if (g_pre_container == NULL)
      return false;
   vhdl_null_stmt *note = new vhdl_null_stmt();
   note->set_comment(text);
   g_pre_container->add_stmt(note);
   return true;
}

/*
 * $value$plusargs(fmt, var): emit, ahead of the current statement,
 *    if sv_value_plusargs(fmt) /= 0 then var := <converted plusarg>; end if;
 * (':=' or '<=' per the target's declaration, as for $random's seed).
 * make_fmt builds a fresh VHDL expression for the format string each call.
 */
bool emit_value_plusargs_pre(ivl_expr_t target, vhdl_expr *(*make_fmt)(ivl_expr_t),
                             ivl_expr_t fmt)
{
   if (g_pre_proc == NULL || g_pre_container == NULL) {
      error("$value$plusargs outside a procedural statement is not supported");
      return false;
   }
   if (in_conditional_eval()) {
      // Its write would run ahead of the statement, every time: wrong when
      // Verilog evaluates the call conditionally (?:, && / ||) or again on
      // each pass (a loop condition)
      error("%s:%d: $value$plusargs in a branch of ?:, an operand of && or "
            "|| or a loop condition has no VHDL translation",
            ivl_expr_file(target), ivl_expr_lineno(target));
      return false;
   }
   if (ivl_expr_type(target) != IVL_EX_SIGNAL) {
      error("$value$plusargs second argument must be a variable");
      return false;
   }
   ivl_signal_t sig = ivl_expr_signal(target);
   string name = get_renamed_signal(sig);
   vhdl_decl *decl = g_pre_proc->get_scope()->get_decl(name);
   if (decl == NULL) {
      error("$value$plusargs: no declaration for %s", name.c_str());
      return false;
   }
   const vhdl_type *t = decl->get_type();
   vhdl_expr *rhs = NULL;
   switch (t->get_name()) {
   case VHDL_TYPE_REAL: {
      vhdl_fcall *f = new vhdl_fcall("sv_plusarg_real", vhdl_type::real());
      f->add_expr(make_fmt(fmt));
      rhs = f;
      break;
   }
   case VHDL_TYPE_LOGIC3D_VECTOR: {
      const int w = t->get_width();
      vhdl_fcall *f = new vhdl_fcall("sv_plusarg_vec", new vhdl_type(*t));
      f->add_expr(make_fmt(fmt));
      f->add_expr(new vhdl_const_int(w));
      rhs = f;
      break;
   }
   default:
      error("$value$plusargs: unsupported target type for %s", name.c_str());
      return false;
   }

   vhdl_fcall *found = new vhdl_fcall("sv_value_plusargs", vhdl_type::integer());
   found->add_expr(make_fmt(fmt));
   vhdl_expr *test = new vhdl_binop_expr(found, VHDL_BINOP_NEQ,
                                         new vhdl_const_int(0),
                                         vhdl_type::boolean());
   vhdl_if_stmt *vif = new vhdl_if_stmt(test);
   vhdl_decl::assign_type_t atype = decl->assignment_type();
   vhdl_var_ref *target_ref = new vhdl_var_ref(name.c_str(), new vhdl_type(*t));
   if (atype == vhdl_decl::ASSIGN_NONBLOCK) {
      // A blocking write of the argument, as make_assignment does one
      if (deposits_signal(g_pre_proc, name, true)) {
         atype = vhdl_decl::ASSIGN_BLOCK;
         g_pre_proc->mark_deposited(name);
      }
      else if (g_pre_proc->get_scope()->allow_signal_assignment())
         g_pre_proc->add_blocking_target(target_ref);
   }
   vif->get_then_container()->add_stmt(assign_for(atype, target_ref, rhs));
   g_pre_container->add_stmt(vif);
   return true;
}

// A Verilog integer argument as vvp reads it (vpiIntVal): its low 32 bits as
// a signed int32 (a narrower value extended by its own signedness)
static vhdl_expr *verilog_int32(ivl_expr_t a)
{
   vhdl_expr *v = translate_expr(a);
   if (v == NULL || v->get_type() == NULL)
      return v;
   vhdl_type integer(VHDL_TYPE_INTEGER);
   const vhdl_type_name_t tn = v->get_type()->get_name();
   if (tn != VHDL_TYPE_LOGIC3D_VECTOR && tn != VHDL_TYPE_LOGIC3D)
      return v->cast(&integer);
   const int w = ivl_expr_width(a) < 1 ? 1 : ivl_expr_width(a);
   vhdl_expr *s32;
   if (ivl_expr_signed(a) && w < 32) {
      vhdl_fcall *s = new vhdl_fcall("l3d_to_signed", vhdl_type::nsigned(w));
      s->add_expr(v);
      s32 = s->resize(32);                       // sign-extend
   }
   else {
      vhdl_fcall *u = new vhdl_fcall("l3d_to_unsigned", vhdl_type::nunsigned(w));
      u->add_expr(v);
      vhdl_fcall *sg = new vhdl_fcall("signed", vhdl_type::nsigned(32));
      sg->add_expr(u->resize(32));               // zero-extend or truncate
      s32 = sg;
   }
   vhdl_fcall *ti = new vhdl_fcall("To_Integer", vhdl_type::integer());
   ti->add_expr(s32);
   return ti;
}

// A VHDL integer as a Verilog 32-bit integer: logic3d_vector(31 downto 0)
static vhdl_expr *int32_to_l3d(vhdl_expr *v)
{
   vhdl_fcall *ts = new vhdl_fcall("to_signed", vhdl_type::nsigned(32));
   ts->add_expr(v);
   ts->add_expr(new vhdl_const_int(32));
   vhdl_fcall *u = new vhdl_fcall("unsigned", vhdl_type::nunsigned(32));
   u->add_expr(ts);
   vhdl_fcall *l3 = new vhdl_fcall("unsigned_to_l3d",
                                   vhdl_type::logic3d_vector(31, 0));
   l3->add_expr(u);
   return l3;
}

/*
 * $dist_uniform(seed, start, end), $dist_normal(seed, mean, std_dev),
 * $dist_exponential(seed, mean), $dist_poisson(seed, mean),
 * $dist_chi_square(seed, df), $dist_t(seed, df), $dist_erlang(seed, k, mean)
 * (IEEE 1364-2005 17.9.2): the sv_math_pkg procedure of the same name
 * (libsv_math.so, the standard's Annex B code that vvp runs), drawn ahead of
 * the current statement on two process integers:
 *
 *    sv_dist_seed_<n> := <seed>;
 *    dist_<kind>(sv_dist_seed_<n>, <arguments>, sv_dist_val_<n>);
 *    <seed> := <sv_dist_seed_<n>>;     -- a blocking assignment of the seed
 *
 * and the call's value is sv_dist_val_<n>. The seed is an inout argument, a
 * variable; every argument goes in as vvp reads it, a 32-bit integer.
 */
vhdl_expr *emit_dist_pre(ivl_expr_t e)
{
   static const struct { const char *name; unsigned nargs; } kinds[] = {
      { "$dist_uniform", 3 }, { "$dist_normal", 3 },
      { "$dist_exponential", 2 }, { "$dist_poisson", 2 },
      { "$dist_chi_square", 2 }, { "$dist_t", 2 }, { "$dist_erlang", 3 },
      { NULL, 0 } };
   const char *name = ivl_expr_name(e);
   const char *file = ivl_expr_file(e);
   const unsigned line = ivl_expr_lineno(e);
   unsigned nargs = 0;
   for (int i = 0; kinds[i].name; i++)
      if (strcmp(name, kinds[i].name) == 0)
         nargs = kinds[i].nargs;
   if (nargs == 0 || !get_sv2vhdl_mode()) {
      error("No translation for system function %s", name);
      return NULL;
   }
   if (ivl_expr_parms(e) != nargs) {
      error("%s:%d: %s takes %u arguments", file, line, name, nargs);
      return NULL;
   }
   if (g_pre_proc == NULL || g_pre_container == NULL) {
      error("%s:%d: %s outside a procedural statement has no VHDL "
            "translation", file, line, name);
      return NULL;
   }
   if (in_conditional_eval()) {
      // Its draw would run ahead of the statement, every time: wrong when
      // Verilog evaluates the call conditionally or again on each pass
      error("%s:%d: %s in a branch of ?:, an operand of && or || or a loop "
            "condition has no VHDL translation", file, line, name);
      return NULL;
   }

   // The seed: an integer variable (not a memory word or a select)
   ivl_expr_t se = ivl_expr_parm(e, 0);
   vhdl_decl *sdecl = NULL;
   std::string sname;
   if (ivl_expr_type(se) == IVL_EX_SIGNAL && ivl_expr_oper1(se) == NULL) {
      ensure_signal_declared(ivl_expr_signal(se));   // a package variable
      if (seen_signal_before(ivl_expr_signal(se))) {
         sname = get_renamed_signal(ivl_expr_signal(se));
         sdecl = g_pre_proc->get_scope()->get_decl(sname);
      }
   }
   if (sdecl == NULL || sdecl->get_type() == NULL
       || sdecl->assignment_type() == vhdl_decl::ASSIGN_CONST) {
      error("%s:%d: the seed of %s must be an integer variable", file, line,
            name);
      return NULL;
   }

   vhdl_expr *seed_in = verilog_int32(se);
   if (seed_in == NULL)
      return NULL;
   std::vector<vhdl_expr*> args;
   for (unsigned i = 1; i < nargs; i++) {
      vhdl_expr *a = verilog_int32(ivl_expr_parm(e, i));
      if (a == NULL)
         return NULL;
      args.push_back(a);
   }

   static int dist_count = 0;
   ostringstream sv, rv;
   sv << "sv_dist_seed_" << ++dist_count;
   rv << "sv_dist_val_" << dist_count;
   vhdl_scope *pscope = g_pre_proc->get_scope();
   pscope->add_decl(new vhdl_var_decl(sv.str(), vhdl_type::integer()));
   pscope->add_decl(new vhdl_var_decl(rv.str(), vhdl_type::integer()));

   vhdl_assign_stmt *in = new vhdl_assign_stmt(
      new vhdl_var_ref(sv.str(), vhdl_type::integer()), seed_in);
   ostringstream cs;
   cs << name << " (" << file << ":" << line << ")";
   in->set_comment(cs.str());
   g_pre_container->add_stmt(in);

   vhdl_pcall_stmt *call = new vhdl_pcall_stmt(name + 1);   // dist_<kind>
   call->add_expr(new vhdl_var_ref(sv.str(), vhdl_type::integer()));
   for (size_t i = 0; i < args.size(); i++)
      call->add_expr(args[i]);
   call->add_expr(new vhdl_var_ref(rv.str(), vhdl_type::integer()));
   g_pre_container->add_stmt(call);

   // The seed comes back: a blocking assignment of the variable, as
   // make_assignment makes one
   vhdl_var_ref *lhs = new vhdl_var_ref(sname, new vhdl_type(*sdecl->get_type()));
   vhdl_expr *back = int32_to_l3d(new vhdl_var_ref(sv.str(), vhdl_type::integer()))
      ->cast(sdecl->get_type());
   vhdl_decl::assign_type_t atype = sdecl->assignment_type();
   if (atype == vhdl_decl::ASSIGN_NONBLOCK) {
      if (deposits_signal(g_pre_proc, sname, true)) {
         atype = vhdl_decl::ASSIGN_BLOCK;
         g_pre_proc->mark_deposited(sname);
      }
      else if (g_pre_proc->get_scope()->allow_signal_assignment())
         g_pre_proc->add_blocking_target(lhs);
   }
   g_pre_container->add_stmt(assign_for(atype, lhs, back));

   vhdl_expr *val = int32_to_l3d(new vhdl_var_ref(rv.str(), vhdl_type::integer()));
   const int w = ivl_expr_width(e);
   if (w != 32 && w >= 1)
      return val->resize(w);
   return val;
}

static int draw_stmt_inner(vhdl_procedural *proc, stmt_container *container,
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
   case IVL_ST_DO_WHILE:
      return draw_do_while(proc, container, stmt);
   case IVL_ST_BREAK:
   case IVL_ST_CONTINUE:
      return draw_break_continue(proc, container, stmt);
   case IVL_ST_FORLOOP:
      return draw_for_loop(proc, container, stmt);
   case IVL_ST_FOREVER:
      return draw_forever(proc, container, stmt);
   case IVL_ST_REPEAT:
      return draw_repeat(proc, container, stmt);
   case IVL_ST_UTASK:
      return draw_utask(proc, container, stmt);
   case IVL_ST_FORCE:
      return draw_force(proc, container, stmt);
   case IVL_ST_RELEASE:
      return draw_release(proc, container, stmt);
   case IVL_ST_DISABLE:
      // Verilog `disable <scope>' and SV `return': leave the enclosing named
      // block, task or function (draw_disable; it was drawn as `null', so
      // the statements after it ran)
      return draw_disable(proc, container, stmt);
   case IVL_ST_ALLOC:
   case IVL_ST_FREE:
      // an automatic task's activation (draw_alloc_free)
      return draw_alloc_free(proc, container, stmt);
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
      return draw_casezx(proc, container, stmt, is_last);
   case IVL_ST_FORK:
   case IVL_ST_FORK_JOIN_ANY:
   case IVL_ST_FORK_JOIN_NONE:
      error("unsupported construct (fork) at %s:%d: a fork statement has no "
            "VHDL translation", ivl_stmt_file(stmt), ivl_stmt_lineno(stmt));
      return 1;
   case IVL_ST_CASSIGN:
      // Procedural continuous assign: approximate with force (see draw_force)
      return draw_force(proc, container, stmt);
   case IVL_ST_DEASSIGN:
      return draw_release(proc, container, stmt);
   default:
      error("No VHDL translation for statement at %s:%d (type = %d)",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt),
            ivl_statement_type(stmt));
      return 1;
   }
}

/*
 * Where seeded call `call' sits in the statement being drawn: in an operand
 * Verilog may leave unevaluated (a ?: branch, the right operand of && or
 * ||: vvp evaluates them lazily), and after a read of its own seed in
 * Verilog's left-to-right evaluation order.  For emit_seeded_random_pre's
 * warnings.
 */
namespace {
struct seed_scan_t {
   ivl_expr_t   call;
   ivl_signal_t seed;
   bool         found, cond, read_before;
   bool         in_loop_test;   // in the test of a while/for/do-while loop
};
}

static bool is_seeded_rng_call(ivl_expr_t e)
{
   if (ivl_expr_type(e) != IVL_EX_SFUNC || ivl_expr_parms(e) < 1)
      return false;
   const char *n = ivl_expr_name(e);
   return strcmp(n, "$random") == 0 || strcmp(n, "$urandom") == 0;
}

static void seed_scan(ivl_expr_t e, seed_scan_t &s, bool cond)
{
   if (e == NULL || s.found)
      return;
   if (e == s.call) {
      s.found = true;
      s.cond = cond;
      return;
   }
   switch (ivl_expr_type(e)) {
   case IVL_EX_SIGNAL:
      if (ivl_expr_signal(e) == s.seed)
         s.read_before = true;
      seed_scan(ivl_expr_oper1(e), s, cond);       // a word index
      return;
   case IVL_EX_BINARY: {
      const char op = ivl_expr_opcode(e);
      seed_scan(ivl_expr_oper1(e), s, cond);
      seed_scan(ivl_expr_oper2(e), s, cond || op == 'a' || op == 'o');
      return;
   }
   case IVL_EX_TERNARY:
      seed_scan(ivl_expr_oper1(e), s, cond);
      seed_scan(ivl_expr_oper2(e), s, true);
      seed_scan(ivl_expr_oper3(e), s, true);
      return;
   case IVL_EX_UNARY:
      seed_scan(ivl_expr_oper1(e), s, cond);
      return;
   case IVL_EX_SELECT:
      seed_scan(ivl_expr_oper1(e), s, cond);
      seed_scan(ivl_expr_oper2(e), s, cond);
      return;
   case IVL_EX_SFUNC:
   case IVL_EX_UFUNC:
   case IVL_EX_CONCAT:
      for (unsigned i = 0; i < ivl_expr_parms(e); i++) {
         // another seeded call's seed is handed over, not read here
         if (i == 0 && is_seeded_rng_call(e))
            continue;
         seed_scan(ivl_expr_parm(e, i), s, cond);
      }
      return;
   default:
      return;
   }
}

static void seed_scan_stmt(ivl_statement_t st, seed_scan_t &s)
{
   switch (ivl_statement_type(st)) {
   case IVL_ST_ASSIGN:
   case IVL_ST_ASSIGN_NB:
      seed_scan(ivl_stmt_rval(st), s, false);
      break;
   case IVL_ST_STASK:
      for (unsigned i = 0; i < ivl_stmt_parm_count(st); i++)
         seed_scan(ivl_stmt_parm(st, i), s, false);
      break;
   case IVL_ST_CONDIT:
   case IVL_ST_CASE:
   case IVL_ST_CASER:
   case IVL_ST_CASEX:
   case IVL_ST_CASEZ:
   case IVL_ST_REPEAT:
      seed_scan(ivl_stmt_cond_expr(st), s, false);
      break;
   case IVL_ST_DO_WHILE:
   case IVL_ST_WHILE:
   case IVL_ST_FORLOOP:
      seed_scan(ivl_stmt_cond_expr(st), s, false);
      s.in_loop_test = s.found;
      break;
   case IVL_ST_DELAYX:
      seed_scan(ivl_stmt_delay_expr(st), s, false);
      break;
   default:
      break;
   }
}

/*
 * $random(seed) (IEEE 1364 17.9.1) and $urandom(seed) (IEEE 1800 18.13.1)
 * draw from the caller's seed variable and leave the advanced seed in it,
 * as vvp's rtl_dist_uniform(&seed, ...) does -- the value and the new seed
 * differ.  Ahead of the statement being drawn:
 *
 *    SV_Random_<n> := sv_random_value(seed);    -- the value, 32 bits
 *                                                -- (sv_urandom_value)
 *    sv_urandom_seed(seed);                      -- $urandom(seed) only: the
 *                                                -- $urandom generator keeps
 *                                                -- the advanced seed
 *    seed := sv_random_next(seed);               -- `<=' as a blocking
 *                                                -- assignment would be
 *
 * and the call reads SV_Random_<n>; NULL after an error.  The calls of a
 * statement draw in turn, in Verilog's left-to-right order (two calls on one
 * seed draw two numbers), and one in a condition, a $display argument or a
 * non-blocking assignment advances its seed as one in a blocking assignment
 * does (draw_while draws a loop test's again before every test; a loop
 * test drawn any other way is an error).  It used to be the glibc-constant
 * LCG sv_random(seed), whose value was the new seed, advanced only by a
 * blocking assignment's.  Two shapes stay approximate, each with a warning:
 * a call Verilog may skip (a ?: branch, the right operand of && or ||)
 * draws all the same, and a read of the seed ahead of the call in the same
 * statement sees the advanced seed.  `call' is the call (for the warnings;
 * NULL for one called as a task).
 */
static int g_seeded_random_count = 0;

// A translator warning once per text (draw_while draws a loop's body twice)
static void warn_once(const std::string &text)
{
   static std::set<std::string> said;
   if (said.insert(text).second)
      cerr << "Warning: " << text << endl;
}

vhdl_expr *emit_seeded_random_pre(const char *fname, ivl_expr_t call,
                                  ivl_expr_t seed, bool urandom,
                                  const char *file, unsigned line)
{
   if (g_pre_proc == NULL || g_pre_container == NULL) {
      error("%s:%u: %s with a seed outside a procedural statement has no "
            "VHDL translation", file, line, fname);
      return NULL;
   }
   ivl_signal_t sig = (seed != NULL && ivl_expr_type(seed) == IVL_EX_SIGNAL
                       && ivl_expr_oper1(seed) == NULL)
      ? ivl_expr_signal(seed) : NULL;
   if (sig == NULL || ivl_signal_type(sig) != IVL_SIT_REG) {
      error("%s:%u: %s's seed must be an integer/time variable or a register",
            file, line, fname);
      return NULL;
   }
   if (ivl_signal_width(sig) < 32) {
      error("%s:%u: %s's seed variable is less than 32 bits (%u)",
            file, line, fname, ivl_signal_width(sig));
      return NULL;
   }
   ensure_signal_declared(sig);
   const string name = get_renamed_signal(sig);
   vhdl_decl *decl = g_pre_proc->get_scope()->get_decl(name);
   if (decl == NULL || decl->get_type() == NULL
       || decl->get_type()->get_name() != VHDL_TYPE_LOGIC3D_VECTOR) {
      error("%s:%u: no VHDL translation for %s with seed %s (not a "
            "logic3d_vector variable here)", file, line, fname,
            ivl_signal_basename(sig));
      return NULL;
   }
   const vhdl_type *st = decl->get_type();
   vhdl_decl::assign_type_t atype = decl->assignment_type();
   if (atype == vhdl_decl::ASSIGN_NONBLOCK
       && !g_pre_proc->get_scope()->allow_signal_assignment()) {
      error("%s:%u: %s's seed %s is a module variable, which a function "
            "cannot assign in VHDL", file, line, fname,
            ivl_signal_basename(sig));
      return NULL;
   }
   if (atype == vhdl_decl::ASSIGN_CONST) {
      // A function's input, a constant in VHDL: shadowed by a variable, as
      // make_assign_lhs shadows one an assignment writes
      const string shadow_name = name + "_Shadow";
      vhdl_var_decl *shadow = new vhdl_var_decl(shadow_name, st);
      shadow->set_initial(new vhdl_var_ref(name, st));
      g_pre_proc->get_scope()->add_decl(shadow);
      rename_signal(sig, shadow_name);
      return emit_seeded_random_pre(fname, call, seed, urandom, file, line);
   }

   ostringstream where;
   where << fname << "(" << ivl_signal_basename(sig) << ") at " << file
         << ":" << line;
   seed_scan_t sc = { call, sig, false, false, false, false };
   if (call != NULL && g_pre_stmt != NULL)
      seed_scan_stmt(g_pre_stmt, sc);
   if (sc.in_loop_test && !g_pre_per_pass) {
      // Ahead of the statement, the draw would run once, not before every
      // test
      error("%s: %s in the test of this loop has no VHDL translation (draw "
            "into a variable before the loop and at the end of its body)",
            where.str().c_str(), fname);
      return NULL;
   }

   // The value: a read that waits for an earlier `<=' to the seed
   vhdl_fcall *value = new vhdl_fcall(urandom ? "sv_urandom_value"
                                              : "sv_random_value",
                                      vhdl_type::logic3d_vector(31, 0));
   value->add_expr(new vhdl_var_ref(name.c_str(), new vhdl_type(*st)));
   if (g_pre_stmt != NULL)
      emit_wait_for_0(g_pre_proc, g_pre_container, g_pre_stmt, value);
   ostringstream tn;
   tn << "SV_Random_" << g_seeded_random_count++;
   vhdl_var_decl *tmp = new vhdl_var_decl(tn.str(),
                                          vhdl_type::logic3d_vector(31, 0));
   g_pre_proc->get_scope()->add_decl(tmp);
   vhdl_assign_stmt *draw = new vhdl_assign_stmt(tmp->make_ref(), value);
   draw->set_comment(where.str());
   g_pre_container->add_stmt(draw);

   if (urandom) {
      vhdl_pcall_stmt *pc = new vhdl_pcall_stmt("sv_urandom_seed");
      pc->add_expr(new vhdl_var_ref(name.c_str(), new vhdl_type(*st)));
      g_pre_container->add_stmt(pc);
   }

   // The advanced seed, written as a blocking assignment is
   vhdl_var_ref *lhs = new vhdl_var_ref(name.c_str(), new vhdl_type(*st));
   vhdl_fcall *next = new vhdl_fcall("sv_random_next", new vhdl_type(*st));
   next->add_expr(new vhdl_var_ref(name.c_str(), new vhdl_type(*st)));
   if (atype == vhdl_decl::ASSIGN_NONBLOCK) {
      // make_assignment's discipline: deposited (and read back at once) where
      // deposits_signal says so, else a `<=' later reads wait for
      if (deposits_signal(g_pre_proc, name, true)) {
         atype = vhdl_decl::ASSIGN_BLOCK;
         g_pre_proc->mark_deposited(name);
      }
      else
         g_pre_proc->add_blocking_target(lhs);
   }
   g_pre_container->add_stmt(assign_for(atype, lhs, next));

   // (worded for bin/iverilog-sv2ghdl, which repeats a "not translated"
   // warning under vamos, where --vamos-strict makes it an error)
   if (sc.found && sc.cond)
      warn_once(where.str() + " is not translated faithfully: Verilog "
                "evaluates it only when its ?:, && or || operand is taken, "
                "the translation always draws and advances "
                + string(ivl_signal_basename(sig)));
   if (sc.found && sc.read_before)
      warn_once(where.str() + " is not translated faithfully: the statement "
                "reads " + string(ivl_signal_basename(sig)) + " ahead of the "
                "call, and the translation reads it already advanced");
   return tmp->make_ref();
}

// `$random(seed);', `$urandom;' -- a system function called as a task
// (iverilog: "Calling system function $random() as a task"): vvp still
// draws, so the seed, or the design-wide generator, advances
static int draw_stask_random(vhdl_procedural *proc, stmt_container *container,
                             ivl_statement_t stmt)
{
   const char *name = ivl_stmt_name(stmt);
   const bool urandom = strcmp(name, "$urandom") == 0;
   ivl_expr_t seed = ivl_stmt_parm_count(stmt) >= 1 ? ivl_stmt_parm(stmt, 0)
                                                    : NULL;
   if (seed != NULL)
      return emit_seeded_random_pre(name, NULL, seed, urandom,
                                    ivl_stmt_file(stmt),
                                    ivl_stmt_lineno(stmt)) ? 0 : 1;
   ostringstream tn;
   tn << "SV_Random_" << g_seeded_random_count++;
   vhdl_var_decl *tmp = new vhdl_var_decl(tn.str(), vhdl_type::integer());
   proc->get_scope()->add_decl(tmp);
   vhdl_assign_stmt *a = new vhdl_assign_stmt(
      tmp->make_ref(),
      new vhdl_fcall(urandom ? "sv_urandom" : "random", vhdl_type::integer()));
   ostringstream c;
   c << name << " at " << ivl_stmt_file(stmt) << ":" << ivl_stmt_lineno(stmt)
     << ", its value dropped";
   a->set_comment(c.str());
   container->add_stmt(a);
   return 0;
}
