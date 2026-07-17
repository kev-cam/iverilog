// -*- mode: c++ -*-
#ifndef INC_VHDL_TARGET_H
#define INC_VHDL_TARGET_H

#include "vhdl_config.h"
#include "ivl_target.h"

#include "support.hh"
#include "vhdl_syntax.hh"

#include <string>

void error(const char *fmt, ...);
void debug_msg(const char *fmt, ...);

int draw_scope(ivl_scope_t scope, void *_parent);
extern "C" int draw_process(ivl_process_t net, void *cd);
int draw_stmt(vhdl_procedural *proc, stmt_container *container,
              ivl_statement_t stmt, bool is_last = false);
int draw_lpm(vhdl_arch *arch, ivl_lpm_t lpm);
void draw_logic(vhdl_arch *arch, ivl_net_logic_t log);
void draw_switches(vhdl_arch *arch, ivl_scope_t scope);

vhdl_expr *translate_expr(ivl_expr_t e);
vhdl_expr *translate_time_expr(ivl_expr_t e);

std::string nexus_to_signal_basename(ivl_nexus_t nex);
std::string analog_expr_to_str(ivl_expr_t expr);
std::string analog_stmt_to_str(ivl_statement_t stmt);

ivl_design_t get_vhdl_design();
// The VHDL type for a signal: real for IVL_VT_REAL, else width/sign-based.
// Every signal/parameter/local declaration site must use this (not raw
// vhdl_type::type_for) or real-typed locals silently become logic3d.
vhdl_type *vhdl_type_for_signal(ivl_signal_t sig);
vhdl_var_ref *nexus_to_var_ref(vhdl_scope *arch_scope, ivl_nexus_t nexus);
// Convert a bit/part/word index expression to a VHDL integer honouring the
// VERILOG signedness of the index (a signed -1 index must become -1, not
// 2**32-1: unsigned to_integer saturates it to integer'high and every
// bounds-guard then misfires).
vhdl_expr *index_to_integer(ivl_expr_t e, vhdl_expr *v);
vhdl_var_ref* readable_ref(vhdl_scope* scope, ivl_nexus_t nex);
std::string make_safe_name(ivl_signal_t sig);
void require_support_function(support_function_t f);

bool is_hoisted_signal(ivl_signal_t sig);
void clear_hoisted_signal(ivl_signal_t sig);

#endif /* #ifndef INC_VHDL_TARGET_H */
