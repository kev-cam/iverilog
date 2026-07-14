/*
 *  Managing global state for the VHDL code generator.
 *
 *  Copyright (C) 2009  Nick Gasson (nick@nickg.me.uk)
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

#ifndef INC_VHDL_STATE_HH
#define INC_VHDL_STATE_HH

#include "ivl_target.h"

#include <iosfwd>
#include <string>

class vhdl_scope;
class vhdl_entity;

// Mapping of Verilog to VHDL signals
bool seen_signal_before(ivl_signal_t sig);
void remember_signal(ivl_signal_t sig, vhdl_scope *scope);
void rename_signal(ivl_signal_t sig, const std::string &renamed);
vhdl_scope *find_scope_for_signal(ivl_signal_t sig);
const std::string &get_renamed_signal(ivl_signal_t sig);
ivl_signal_t find_signal_named(const std::string &name, const vhdl_scope *scope);

// Manage the set of VHDL entities
void remember_entity(vhdl_entity *ent, ivl_scope_t scope);
vhdl_entity* find_entity(ivl_scope_t scope);
vhdl_entity* find_entity(const std::string& name);
void emit_all_entities(std::ostream& os, int max_depth);
void free_all_vhdl_objects();

// Get and set the active entity
vhdl_entity *get_active_entity();
void set_active_entity(vhdl_entity *ent);

// Scope-keyed store of per-scope facts (timescale, hierarchical name) so that
// deeply-nested translation (e.g. $time, %m) can look up the scope it is in
// without threading the ivl_scope through every call. set_active_scope() both
// records the scope's data and marks it current.
void set_active_scope(ivl_scope_t scope);
ivl_scope_t get_active_scope();
// Time units of the active scope as a signed power of 10 (e.g. -9 = 1 ns),
// per ivl_scope_time_units; 0 if no active scope.
int active_time_units();
// Time precision of the active scope as a signed power of 10; 0 if none.
int active_time_precision();
// Verilog hierarchical name of the active scope (ivl_scope_name); "" if none.
std::string active_hier_name();

// Manage mapping of scopes to a single VHDL entity
bool is_default_scope_instance(ivl_scope_t s);
bool seen_this_scope_type(ivl_scope_t s);

// sv2vhdl mode flag
void set_sv2vhdl_mode(bool mode);
bool get_sv2vhdl_mode();

#endif  // #ifndef INC_VHDL_STATE_HH
