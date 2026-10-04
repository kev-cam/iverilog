/*
 *  VHDL code generation for the Verilog file I/O system tasks and functions
 *  (sv2vhdl mode):
 *
 *    $fopen, $fopenr, $fopenw, $fopena                       (functions)
 *    $fdisplay, $fwrite, $fstrobe, $fmonitor and their b/h/o forms,
 *    $fclose, $fflush                                        (tasks)
 *
 *  and the argument helpers draw_stask_readmem (stmt.cc) shares for
 *  $readmemh, $readmemb, $writememh and $writememb.
 *
 *  They run on the file I/O runtime of the sv2vhdl library (nvc
 *  lib/sv2vhdl: sv_display_pkg's sv_fopen, sv_fdisplay, sv_fstrobe_arm, ...;
 *  logic3d_types_pkg's sv_vec_int32, sv_vec_str and, for the memory tasks,
 *  sv_readmem_* and sv_writemem_*), which follows vvp: a descriptor is the
 *  argument's vpiIntVal (MCD bit 0 stdout, FD bit 31), the messages are
 *  vvp's, and text for stdout goes through the $display line buffer.  The
 *  formatting is the $display machinery's (build_display_text in stmt.cc).
 *
 *  A form with no translation -- a call in a VHDL function that would have
 *  to assign a signal, a memory in another module -- is left out with the
 *  located comment "Unsupported system task <name> omitted here
 *  (<file>:<line>)" every untranslated task gets (vamos reports it: a
 *  warning under vcs, an error under vcs-ams), never silently.
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
 */

#include "vhdl_target.h"
#include "state.hh"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// stmt.cc: the $display text of a task's arguments from first_parm on (dflt:
// the format of an argument with no format of its own), and the wait-for-0
// a read of a blocking target needs first
vhdl_expr *build_display_text(vhdl_procedural *proc, stmt_container *container,
                              ivl_statement_t stmt, int first_parm, char dflt);
void fileio_wait_for_0(vhdl_procedural *proc, stmt_container *container,
                       ivl_statement_t stmt, vhdl_expr *expr);
// stmt.cc: the signals a $monitor-style companion watches
std::vector<std::string> monitor_watch(vhdl_process *mon, vhdl_scope *ascope,
                                       vhdl_expr *text, const std::string &skip);
// expr.cc
std::string ivl_string_unescape(const char *str);

string fileio_loc(ivl_statement_t stmt)
{
   ostringstream ss;
   ss << ivl_stmt_file(stmt) << ":" << ivl_stmt_lineno(stmt);
   return ss.str();
}

static string expr_loc(ivl_expr_t e)
{
   ostringstream ss;
   const char *f = ivl_expr_file(e);
   ss << (f ? f : "?") << ":" << ivl_expr_lineno(e);
   return ss.str();
}

// The located comment of a task with no translation (draw_stask gives every
// other untranslated task the same), and the reason in the translation log
int fileio_unsupported(stmt_container *container, ivl_statement_t stmt,
                       const char *why)
{
   const char *name = ivl_stmt_name(stmt);
   vhdl_seq_stmt *result = new vhdl_null_stmt();
   ostringstream ss;
   ss << "Unsupported system task " << name << " omitted here ("
      << fileio_loc(stmt) << ")";
   result->set_comment(ss.str());
   container->add_stmt(result);
   cerr << "Warning: no VHDL translation for system task " << name << " at "
        << fileio_loc(stmt) << ": " << why << endl;
   return 0;
}

/*
 * An argument's vpiIntVal (a descriptor, a start or finish address) as a
 * VHDL integer: the low 32 bits, two's complement, x and z bits as 0, a
 * narrower signed value sign-extended (vvp's format_vpiIntVal); a real
 * rounded.  A constant is folded here.  proc NULL: no wait-for-0 (an
 * expression context, whose statement emits it).
 */
vhdl_expr *fileio_int32(vhdl_procedural *proc, stmt_container *container,
                        ivl_statement_t stmt, ivl_expr_t e)
{
   if (ivl_expr_type(e) == IVL_EX_NUMBER) {
      const char *bits = ivl_expr_bits(e);
      const unsigned w = ivl_expr_width(e);
      const bool sext = ivl_expr_signed(e) && w > 0 && w < 32
         && bits[w - 1] == '1';
      uint32_t v = 0;
      for (unsigned i = 0; i < 32; i++) {
         const bool one = i < w ? bits[i] == '1' : sext;
         if (one)
            v |= (uint32_t)1 << i;
      }
      const int32_t iv = (int32_t)v;
      if (iv != INT32_MIN)               // that one has no VHDL literal
         return new vhdl_const_int(iv);
   }
   vhdl_expr *base = translate_expr(e);
   if (base == NULL)
      return NULL;
   if (proc != NULL)
      fileio_wait_for_0(proc, container, stmt, base);
   const vhdl_type *t = base->get_type();
   const vhdl_type_name_t tn = t ? t->get_name() : VHDL_TYPE_INTEGER;
   int w = ivl_expr_width(e);
   if (w < 1)
      w = 1;
   switch (tn) {
   case VHDL_TYPE_INTEGER:
      return base;
   case VHDL_TYPE_REAL: {
      vhdl_fcall *f = new vhdl_fcall("integer", vhdl_type::integer());
      f->add_expr(base);
      return f;
   }
   case VHDL_TYPE_BOOLEAN:
      base = base->cast(vhdl_type::logic3d());
      break;
   case VHDL_TYPE_LOGIC3D:
   case VHDL_TYPE_LOGIC3D_VECTOR:
      break;
   case VHDL_TYPE_SIGNED:
   case VHDL_TYPE_UNSIGNED:
   case VHDL_TYPE_STD_LOGIC_VECTOR: {
      vhdl_type l3(VHDL_TYPE_LOGIC3D_VECTOR, w - 1, 0);
      base = base->cast(&l3);
      break;
   }
   default:
      error("%s:%d: no translation for this argument of %s (a %s)",
            ivl_stmt_file(stmt), ivl_stmt_lineno(stmt), ivl_stmt_name(stmt),
            t ? t->get_string().c_str() : "?");
      return NULL;
   }
   vhdl_fcall *f = new vhdl_fcall("sv_vec_int32", vhdl_type::integer());
   f->add_expr(base);
   f->add_expr(new vhdl_var_ref(ivl_expr_signed(e) ? "true" : "false",
                                vhdl_type::boolean()));
   return f;
}

/*
 * A file name or mode argument as a VHDL string: a literal as itself, any
 * other value as vvp's vpiStringVal of it (sv_vec_str).
 */
vhdl_expr *fileio_string(vhdl_procedural *proc, stmt_container *container,
                         ivl_statement_t stmt, ivl_expr_t e)
{
   if (ivl_expr_type(e) == IVL_EX_STRING)
      return new vhdl_const_string(ivl_string_unescape(ivl_expr_string(e)));
   vhdl_expr *base = translate_expr(e);
   if (base == NULL)
      return NULL;
   if (proc != NULL)
      fileio_wait_for_0(proc, container, stmt, base);
   const vhdl_type *t = base->get_type();
   const vhdl_type_name_t tn = t ? t->get_name() : VHDL_TYPE_INTEGER;
   if (tn == VHDL_TYPE_STRING)
      return base;
   if (tn != VHDL_TYPE_LOGIC3D && tn != VHDL_TYPE_LOGIC3D_VECTOR) {
      int w = ivl_expr_width(e);
      if (w < 1)
         w = 1;
      vhdl_type l3(VHDL_TYPE_LOGIC3D_VECTOR, w - 1, 0);
      base = base->cast(&l3);
   }
   vhdl_fcall *f = new vhdl_fcall("sv_vec_str", vhdl_type::string());
   f->add_expr(base);
   return f;
}

// vpi_get_str(vpiType, <argument>) as vvp has it, for the "file name
// argument (<kind>) is not a valid string" message
const char *fileio_vpi_kind(ivl_expr_t e)
{
   switch (ivl_expr_type(e)) {
   case IVL_EX_STRING:
   case IVL_EX_NUMBER:
   case IVL_EX_REALNUM:
      return ivl_expr_parameter(e) ? "vpiParameter" : "vpiConstant";
   case IVL_EX_SIGNAL: {
      ivl_signal_t sig = ivl_expr_signal(e);
      if (ivl_expr_oper1(e) != NULL)
         return "vpiMemoryWord";
      if (ivl_signal_data_type(sig) == IVL_VT_REAL)
         return "vpiRealVar";
      if (ivl_signal_data_type(sig) == IVL_VT_STRING)
         return "vpiStringVar";
      if (ivl_signal_type(sig) != IVL_SIT_REG)
         return "vpiNet";
      if (ivl_signal_integer(sig))
         return "vpiIntegerVar";
      return "vpiReg";
   }
   case IVL_EX_SELECT:
      return "vpiPartSelect";
   case IVL_EX_SFUNC:
      return "vpiSysFuncCall";
   case IVL_EX_UFUNC:
      return "vpiFuncCall";
   default:
      return "vpiOperation";
   }
}

// vvp warns about a start/finish address that is a real constant,
// parameter or variable (get_mem_params)
bool fileio_real_arg(ivl_expr_t e)
{
   if (e == NULL)
      return false;
   if (ivl_expr_type(e) == IVL_EX_REALNUM)
      return true;
   return ivl_expr_type(e) == IVL_EX_SIGNAL && ivl_expr_oper1(e) == NULL
      && ivl_signal_data_type(ivl_expr_signal(e)) == IVL_VT_REAL;
}

// name is base, or base followed by one of b h o (*dflt: that letter, or
// 'd' for base itself)
static bool name_is(const char *name, const char *base, char *dflt)
{
   const size_t n = strlen(base);
   if (strncmp(name, base, n) != 0)
      return false;
   if (name[n] == 0) {
      *dflt = 'd';
      return true;
   }
   if ((name[n] == 'b' || name[n] == 'h' || name[n] == 'o') && name[n + 1] == 0) {
      *dflt = name[n];
      return true;
   }
   return false;
}

bool fileio_task(const char *name)
{
   char d;
   return name_is(name, "$fdisplay", &d) || name_is(name, "$fwrite", &d)
      || name_is(name, "$fstrobe", &d) || name_is(name, "$fmonitor", &d)
      || strcmp(name, "$fclose") == 0 || strcmp(name, "$fflush") == 0;
}

// The descriptor argument (parameter 0) of a task
static vhdl_expr *fd_param(vhdl_procedural *proc, stmt_container *container,
                           ivl_statement_t stmt)
{
   ivl_expr_t fe = ivl_stmt_parm_count(stmt) > 0 ? ivl_stmt_parm(stmt, 0)
                                                  : NULL;
   if (fe == NULL) {
      error("%s:%d: %s needs a file descriptor argument", ivl_stmt_file(stmt),
            ivl_stmt_lineno(stmt), ivl_stmt_name(stmt));
      return NULL;
   }
   return fileio_int32(proc, container, stmt, fe);
}

/*
 * $fdisplay / $fwrite (and their b/h/o forms):
 *
 *    sv_fdisplay(<fd>, <text>, "$fdisplay", "<file>:<line>");
 *
 * The runtime warns about an invalid descriptor and is silent for 0; the
 * text is evaluated either way, as vvp evaluates the arguments (a $random
 * among them is drawn).
 */
static int draw_fwrite(vhdl_procedural *proc, stmt_container *container,
                       ivl_statement_t stmt, bool newline, char dflt)
{
   vhdl_expr *fd = fd_param(proc, container, stmt);
   if (fd == NULL)
      return 1;
   vhdl_expr *text;
   if (ivl_stmt_parm_count(stmt) < 2)
      text = new vhdl_const_string("");
   else if ((text = build_display_text(proc, container, stmt, 1, dflt)) == NULL)
      return 1;
   vhdl_pcall_stmt *pc = new vhdl_pcall_stmt(newline ? "sv_fdisplay" : "sv_fwrite");
   pc->add_expr(fd);
   pc->add_expr(text);
   pc->add_expr(new vhdl_const_string(ivl_stmt_name(stmt)));
   pc->add_expr(new vhdl_const_string(fileio_loc(stmt)));
   container->add_stmt(pc);
   return 0;
}

/*
 * $fstrobe / $fmonitor (and their b/h/o forms): a companion POSTPONED
 * process writes at the end of the time step, as $strobe's and $monitor's
 * do (stmt.cc).  The call checks the descriptor and records the call in the
 * runtime under the statement's key -- the 'path_name of its request
 * signal, so every instance of a module has its own -- and wakes the
 * companion:
 *
 *    sv_fstrobe_arm(sv_fstrobe_req_<n>'path_name, <fd>, "$fstrobe", "<loc>");
 *    sv_fstrobe_req_<n> <= sv_fstrobe_req_<n> + 1;
 *
 *    sv_fstrobe_<n>: postponed process (sv_fstrobe_req_<n>) is
 *    begin
 *       while sv_fstrobe_next(sv_fstrobe_req_<n>'path_name) loop
 *          sv_fput_due(<text> & LF);
 *       end loop;
 *    end process;
 *
 * Each call due is written once, to its own descriptor, with the text
 * evaluated for it (vvp's strobe_cb / monitor_cb_2); a descriptor closed
 * meanwhile is skipped.  $fmonitor's companion is also sensitive to every
 * signal its text reads -- a constant word or part select of one by itself
 * -- and passes whether one of them changed in this time step:
 *
 *    while sv_fmonitor_next(sv_fmonitor_req_<n>'path_name,
 *                           (a'last_event = 0 fs) or ...) loop
 *
 * so every running $fmonitor of the statement writes then, and one called
 * in this time step writes its first line.  An $fclose of a descriptor a
 * monitor writes to ends it (vvp's sys_monitor_fclose); $monitoroff and
 * $monitoron leave $fmonitor alone.
 */
static int g_fio_count = 0;

static int draw_fstrobe(vhdl_procedural *proc, stmt_container *container,
                        ivl_statement_t stmt, bool is_monitor, char dflt)
{
   vhdl_entity *ent = get_active_entity();
   if (ent == NULL) {
      error("%s:%d: %s outside an entity context", ivl_stmt_file(stmt),
            ivl_stmt_lineno(stmt), ivl_stmt_name(stmt));
      return 1;
   }
   if (!proc->get_scope()->allow_signal_assignment())
      return fileio_unsupported(container, stmt, "called where VHDL cannot "
                                "assign a signal (a function)");
   vhdl_arch *arch = ent->get_arch();
   vhdl_scope *ascope = arch->get_scope();
   const int id = ++g_fio_count;
   const string kind = is_monitor ? "fmonitor" : "fstrobe";
   ostringstream nm;
   nm << id;
   const string req = "sv_" + kind + "_req_" + nm.str();
   const string key = req + "'path_name";

   vhdl_expr *fd = fd_param(proc, container, stmt);
   if (fd == NULL)
      return 1;

   // The call
   vhdl_signal_decl *rd = new vhdl_signal_decl(req, vhdl_type::integer());
   rd->set_initial(new vhdl_const_int(0));
   ascope->add_decl(rd);
   vhdl_pcall_stmt *arm = new vhdl_pcall_stmt(("sv_" + kind + "_arm").c_str());
   arm->add_expr(new vhdl_var_ref(key, vhdl_type::string()));
   arm->add_expr(fd);
   arm->add_expr(new vhdl_const_string(ivl_stmt_name(stmt)));
   arm->add_expr(new vhdl_const_string(fileio_loc(stmt)));
   container->add_stmt(arm);
   container->add_stmt(new vhdl_nbassign_stmt(
      new vhdl_var_ref(req, vhdl_type::integer()),
      new vhdl_binop_expr(new vhdl_var_ref(req, vhdl_type::integer()),
                          VHDL_BINOP_ADD, new vhdl_const_int(1),
                          vhdl_type::integer())));

   // The companion process
   vhdl_process *mon = new vhdl_process(("sv_" + kind + "_" + nm.str()).c_str());
   mon->set_postponed();
   vhdl_expr *text;
   if (ivl_stmt_parm_count(stmt) < 2)
      text = new vhdl_const_string("");
   else if ((text = build_display_text(NULL, mon->get_container(), stmt, 1,
                                       dflt)) == NULL)
      return 1;
   vhdl_binop_expr *line = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
                                               vhdl_type::string());
   line->add_expr(text);
   line->add_expr(new vhdl_var_ref("LF", vhdl_type::string()));
   vhdl_pcall_stmt *out = new vhdl_pcall_stmt("sv_fput_due");
   out->add_expr(line);

   vhdl_fcall *next = new vhdl_fcall("sv_" + kind + "_next", vhdl_type::boolean());
   next->add_expr(new vhdl_var_ref(key, vhdl_type::string()));
   mon->add_sensitivity(req);
   if (is_monitor) {
      vector<string> watched = monitor_watch(mon, ascope, line, req);
      string changed;
      for (size_t i = 0; i < watched.size(); i++) {
         if (!changed.empty())
            changed += " or ";
         changed += "(" + watched[i] + "'last_event = 0 fs)";
      }
      next->add_expr(new vhdl_var_ref(changed.empty() ? "false" : changed,
                                      vhdl_type::boolean()));
   }
   vhdl_while_stmt *loop = new vhdl_while_stmt(next);
   loop->get_container()->add_stmt(out);
   mon->get_container()->add_stmt(loop);
   arch->add_stmt(mon);
   return 0;
}

static int draw_fclose(vhdl_procedural *proc, stmt_container *container,
                       ivl_statement_t stmt)
{
   vhdl_expr *fd = fd_param(proc, container, stmt);
   if (fd == NULL)
      return 1;
   vhdl_pcall_stmt *pc = new vhdl_pcall_stmt("sv_fclose");
   pc->add_expr(fd);
   pc->add_expr(new vhdl_const_string(fileio_loc(stmt)));
   container->add_stmt(pc);
   return 0;
}

static int draw_fflush(vhdl_procedural *proc, stmt_container *container,
                       ivl_statement_t stmt)
{
   if (ivl_stmt_parm_count(stmt) == 0 || ivl_stmt_parm(stmt, 0) == NULL) {
      container->add_stmt(new vhdl_pcall_stmt("sv_fflush_all"));
      return 0;
   }
   vhdl_expr *fd = fd_param(proc, container, stmt);
   if (fd == NULL)
      return 1;
   vhdl_pcall_stmt *pc = new vhdl_pcall_stmt("sv_fflush");
   pc->add_expr(fd);
   pc->add_expr(new vhdl_const_string(fileio_loc(stmt)));
   container->add_stmt(pc);
   return 0;
}

int draw_stask_fileio(vhdl_procedural *proc, stmt_container *container,
                      ivl_statement_t stmt)
{
   const char *name = ivl_stmt_name(stmt);
   char dflt = 'd';
   if (name_is(name, "$fdisplay", &dflt))
      return draw_fwrite(proc, container, stmt, true, dflt);
   if (name_is(name, "$fwrite", &dflt))
      return draw_fwrite(proc, container, stmt, false, dflt);
   if (name_is(name, "$fstrobe", &dflt))
      return draw_fstrobe(proc, container, stmt, false, dflt);
   if (name_is(name, "$fmonitor", &dflt))
      return draw_fstrobe(proc, container, stmt, true, dflt);
   if (strcmp(name, "$fclose") == 0)
      return draw_fclose(proc, container, stmt);
   return draw_fflush(proc, container, stmt);
}

/*
 * $fopen(name [, mode]) and $fopenr/$fopenw/$fopena(name): the descriptor
 * sv_fopen returns (an MCD without a mode, else an FD; 0 on failure), as the
 * call's value of its width.
 */
vhdl_expr *fileio_fopen(ivl_expr_t e)
{
   const char *name = ivl_expr_name(e);
   const unsigned np = ivl_expr_parms(e);
   ivl_expr_t fe = np >= 1 ? ivl_expr_parm(e, 0) : NULL;
   if (fe == NULL) {
      error("%s: %s needs a file name argument", expr_loc(e).c_str(), name);
      return NULL;
   }
   vhdl_expr *fname = fileio_string(NULL, NULL, NULL, fe);
   if (fname == NULL)
      return NULL;
   vhdl_fcall *f = new vhdl_fcall("sv_fopen", vhdl_type::integer());
   f->add_expr(fname);
   f->add_expr(new vhdl_const_string(fileio_vpi_kind(fe)));
   if (strcmp(name, "$fopen") == 0 && (np < 2 || ivl_expr_parm(e, 1) == NULL))
      f->add_expr(new vhdl_const_string(expr_loc(e)));      // an MCD
   else {
      vhdl_expr *mode;
      if (strcmp(name, "$fopen") == 0) {
         mode = fileio_string(NULL, NULL, NULL, ivl_expr_parm(e, 1));
         if (mode == NULL)
            return NULL;
      }
      else
         mode = new vhdl_const_string(string(1, name[strlen(name) - 1]));
      f->add_expr(mode);
      f->add_expr(new vhdl_const_string(name));
      f->add_expr(new vhdl_const_string(expr_loc(e)));
   }
   int w = ivl_expr_width(e);
   if (w < 1)
      w = 32;
   vhdl_fcall *l3 = new vhdl_fcall("to_l3d", vhdl_type::logic3d_vector(w - 1, 0));
   l3->add_expr(f);
   l3->add_expr(new vhdl_const_int(w));
   return l3;
}
