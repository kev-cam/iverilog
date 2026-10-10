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
void report_unplaced_pulls();
extern "C" int draw_process(ivl_process_t net, void *cd);
int draw_stmt(vhdl_procedural *proc, stmt_container *container,
              ivl_statement_t stmt, bool is_last = false);
int draw_lpm(vhdl_arch *arch, ivl_lpm_t lpm);
void draw_logic(vhdl_arch *arch, ivl_net_logic_t log);
void emit_strength_buf(vhdl_arch *arch, vhdl_expr *y, vhdl_expr *data,
                       ivl_drive_t d1, ivl_drive_t d0, const char *basename);
void draw_switches(vhdl_arch *arch, ivl_scope_t scope);

vhdl_expr *translate_expr(ivl_expr_t e);
bool emit_value_plusargs_pre(ivl_expr_t target, vhdl_expr *(*make_fmt)(ivl_expr_t),
                             ivl_expr_t fmt);
// $dist_* (stmt.cc): the draw goes ahead of the current statement; returns
// the call's value
vhdl_expr *emit_dist_pre(ivl_expr_t e);
// An expression Verilog evaluates conditionally or repeatedly -- a branch of
// ?:, the right operand of && or ||, a loop condition -- is translated between
// these two (expr.cc): a system function whose side effect becomes a
// statement ahead of the current one ($dist_*'s seed, $value$plusargs's
// variable) cannot be translated there, and says so.
void begin_conditional_eval();
void end_conditional_eval();
bool in_conditional_eval();
// $random(seed) / $urandom(seed) (stmt.cc): ahead of the statement being drawn,
// draw from the seed variable and advance it as vvp does; the value drawn, a
// 32-bit logic3d_vector (NULL after an error).  `call' is the call (NULL when
// it is called as a task)
vhdl_expr *emit_seeded_random_pre(const char *fname, ivl_expr_t call,
                                  ivl_expr_t seed, bool urandom,
                                  const char *file, unsigned line);
vhdl_expr *translate_time_expr(ivl_expr_t e);

std::string nexus_to_signal_basename(ivl_nexus_t nex);
std::string analog_expr_to_str(ivl_expr_t expr);
std::string analog_stmt_to_str(ivl_statement_t stmt);

ivl_design_t get_vhdl_design();
// The VHDL type for a signal: real for IVL_VT_REAL, else width/sign-based.
// Every signal/parameter/local declaration site must use this (not raw
// vhdl_type::type_for) or real-typed locals silently become logic3d.
vhdl_type *vhdl_type_for_signal(ivl_signal_t sig);
// Declare a package/$unit-scope signal in the active entity on first use.
void ensure_signal_declared(ivl_signal_t sig);
// Emit a function scope's VHDL function into a specific entity (used to draw
// package functions on demand into whichever entity first calls them).
int draw_function_in_entity(ivl_scope_t scope, vhdl_entity *ent);
// A SystemVerilog void function (no result port): drawn and called like a task
bool is_void_function(ivl_scope_t scope);
// The VHDL name of a function scope: its Verilog name plus the flattening
// suffix of an enclosing generate block (see scope.cc).
std::string vhdl_function_name(ivl_scope_t fscope);
vhdl_var_ref *nexus_to_var_ref(vhdl_scope *arch_scope, ivl_nexus_t nexus);
bool nexus_visible_in_scope(vhdl_scope *scope, ivl_nexus_t nexus);
// ICG2EN (stmt.cc): entity-split signature + site clock repointing
bool icg2en_key_enabled();
std::string icg2en_scope_signature(ivl_scope_t scope);
bool icg2en_site_root(ivl_signal_t child_port, ivl_nexus_t *root_out);
void seen_nexus(ivl_nexus_t nexus);
void icg2en_add_entity_ports(ivl_scope_t scope, vhdl_entity *ent);
void icg2en_map_enables(ivl_scope_t child, const vhdl_entity *parent,
                        vhdl_comp_inst *inst);
void icg2en_note_label(ivl_scope_t scope, const std::string &label);
// Convert a bit/part/word index expression to a VHDL integer honouring the
// VERILOG signedness of the index (a signed -1 index must become -1, not
// 2**32-1: unsigned to_integer saturates it to integer'high and every
// bounds-guard then misfires).
vhdl_expr *index_to_integer(ivl_expr_t e, vhdl_expr *v);
// Same for an index that is a net (an LPM part-select base): the caller
// supplies the signedness it derived from the nexus.
vhdl_expr *index_to_integer(vhdl_expr *v, bool is_signed);
vhdl_var_ref* readable_ref(vhdl_scope* scope, ivl_nexus_t nex);
std::string make_safe_name(ivl_signal_t sig);
void replace_consecutive_underscores(std::string& str);
bool is_vhdl_reserved_word(const std::string& word);
// The core's transparent buffer between an input port and a variable or
// expression actual: drawn in the parent, per instance (scope.cc)
bool is_input_port_buffer(ivl_net_logic_t log);
// A core node between such a buffer and the port: the pad, prune or
// instance-array split of the actual, drawn in the parent too (scope.cc)
bool is_input_port_network_lpm(ivl_lpm_t lpm);
// A buffer of that kind the translation cannot draw: reports the error
bool untranslated_port_buffer(ivl_net_logic_t log);
// Whether the one-way copy drawn for part-select tran `sw' needs the
// "connected one way only" warning (scope.cc, T2)
bool tran_vp_copy_needs_warning(vhdl_scope *sc, ivl_switch_t sw);
// A part-select tran the port map draws (an inout port on a concatenation)
bool tran_vp_drawn_by_port_map(ivl_switch_t sw);
// A real signal or constant on the nexus: real temporaries, real arithmetic
bool nexus_is_real(ivl_nexus_t nex);
void require_support_function(support_function_t f);
// disable / SV return (stmt.cc): a function body is drawn between these two,
// so a disable of the function inside it is `return <result>;' and the
// disable targets of the caller (a process drawing a package function on
// demand) are out of its reach
void begin_function_disables(ivl_scope_t fscope, const std::string &result);
void end_function_disables();
// The Verilog process draw_process is drawing (process.cc); NULL outside one
ivl_process_t get_active_ivl_process();

bool is_hoisted_signal(ivl_signal_t sig);
void clear_hoisted_signal(ivl_signal_t sig);

// Whether the array type of that name has its bounds-safe word reader,
// <type>_Rd(memory, index) (scope.cc, declare_word_read)
bool has_word_read(const std::string &array_type_name);

// Verilog's time-zero order (stmt.cc; off with SV2VHDL_TC08=0): an `always
// @(a or b)' (any-edge events, no time-zero trigger) waits for its first event, as
// vvp starts it; an initial block runs after every process has reached its
// first wait, and is not hoisted into declaration initial values, so its
// time-zero assignments are events (an SV variable initializer stays
// hoisted: no event).
bool time_zero_order_enabled();
// Once every process is drawn: drop the initial blocks' time-zero waits when
// no initial block assigns a signal at time zero (process.cc)
void settle_time_zero_waits();

// Named-block locals another process names: found before the processes are
// drawn (process.cc) and declared as architecture signals (scope.cc)
extern "C" int scan_shared_block_locals(ivl_process_t proc, void *);
void hoist_shared_block_locals();
void hoist_block_local(ivl_signal_t sig);

// A net that a force or a release names (stmt.cc): its continuous
// assignment keeps its own driver instead of joining a fused comb cone
// (process.cc, fuse_comb_processes)
void note_forced_net(ivl_signal_t sig);

// %m (stmt.cc): the Verilog name of the active scope, a string expression --
// a constant, or, in a module instantiated more than once, the instance's
// name found at run time from the entity's 'PATH_NAME (SV_Hier_Name);
// declare_hier_names, once every process is drawn, declares SV_Hier_Name in
// each architecture that uses it. instance_vhdl_path (scope.cc): a module
// instance's VHDL path below its design root, ":<label>:...:<label>:".
vhdl_expr *hier_name_expr();
void declare_hier_names();
// The processes that write each variable, before any process is drawn
// (stmt.cc): a nonblocking assignment of its only writer lands as an NBA
extern "C" int census_writers(ivl_process_t p, void *);
bool instance_vhdl_path(ivl_scope_t inst, std::string &path);

#endif /* #ifndef INC_VHDL_TARGET_H */
