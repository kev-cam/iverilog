/*
 *  VHDL code generation for scopes.
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

#include <iostream>
#include <sstream>
#include <cassert>
#include <cctype>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <map>
#include <set>
#include <vector>

using namespace std;

/*
 * This represents the portion of a nexus that is visible within
 * a VHDL scope. If that nexus portion does not contain a signal,
 * then `tmpname' gives the name of the temporary that will be
 * used when this nexus is used in `scope' (e.g. for LPMs that
 * appear in instantiations). The list `connect' lists all the
 * signals that should be joined together to re-create the net.
 */
struct scope_nexus_t {
   vhdl_scope *scope;
   ivl_signal_t sig;            // A real signal
   unsigned pin;                // The pin this signal is connected to
   string tmpname;              // A new temporary signal
   list<ivl_signal_t> connect;  // Other signals to wire together
};

/*
 * This structure is stored in the private part of each nexus.
 * It stores a scope_nexus_t for each VHDL scope which is
 * connected to that nexus. It's stored as a list so we can use
 * contained_within to allow several nested scopes to reference
 * the same signal.
 */
struct const_drv_t {
   vhdl_expr  *expr;
   ivl_drive_t drive0, drive1;
   // Raw constant bit chars (LSB first, from ivl_const_bits): the
   // per-bit strength-buffer emission for vector constants needs the
   // individual bit values, not the packed vhdl expression
   string      bits;
};

struct nexus_private_t {
   list<scope_nexus_t> signals;
   vhdl_expr *const_driver;
   // Drive strengths of the constant driver's nexus pointer: a
   // strength-spec assign with a constant r-value has no BUFZ device,
   // so the strengths live only here
   ivl_drive_t const_drive0 = IVL_DR_STRONG;
   ivl_drive_t const_drive1 = IVL_DR_STRONG;
   string const_bits;          // raw bit chars of const_driver (LSB first)
   // Second and later constant drivers on the same nexus (opposing
   // strength-spec assigns): the single const_driver slot kept only
   // the last one and silently dropped the rest
   list<const_drv_t> const_extra;
   bool has_inout = false;     // nexus touches an inout port => bidirectional
   string inout_module;        // module type of that inout (for origin markup)
   // sv2vhdl mode: a tri1/tri0 net carries a pull ('1' / '0', 0 = none),
   // drawn once as sv_pullup/sv_pulldown instance(s) on the first plain
   // signal of the nexus (draw_constant_drivers), whether or not anything
   // else drives the net. tri_default marks a const_driver that only
   // stands for that pull on an otherwise undriven net.
   char pull = 0;
   bool tri_default = false;
};

// Nexuses whose tri1/tri0 pull is still to be drawn (report_unplaced_pulls)
static list<ivl_nexus_t> g_pull_nexus;

/*
 * Returns the scope_nexus_t of this nexus visible within scope.
 */
static scope_nexus_t *visible_nexus(nexus_private_t *priv, const vhdl_scope *scope)
{
   list<scope_nexus_t>::iterator it;
   for (it = priv->signals.begin(); it != priv->signals.end(); ++it) {
      if (scope->contained_within((*it).scope))
         return &*it;
   }
   return NULL;
}

/*
 * Remember that a signal in `scope' is part of this nexus. The
 * first signal passed to this function for a scope will be used
 * as the canonical representation of this nexus when we need to
 * convert it to a variable reference (e.g. in a LPM input/output).
 */
static void link_scope_to_nexus_signal(nexus_private_t *priv, vhdl_scope *scope,
                                       ivl_signal_t sig, unsigned pin)
{
   scope_nexus_t *sn;
   if ((sn = visible_nexus(priv, scope))) {
      assert(sn->tmpname == "");

      // Remember to connect this signal up later
      // If one of the signals is a input, make sure the input is not being driven
      if (ivl_signal_port(sn->sig) == IVL_SIP_INPUT)
         sn->sig = sig;
      else
         sn->connect.push_back(sig);
   }
   else {
      scope_nexus_t new_sn = { scope, sig, pin, "", list<ivl_signal_t>() };
      priv->signals.push_back(new_sn);
   }
}

/*
 * Make a temporary the representative of this nexus in scope.
 */
static void link_scope_to_nexus_tmp(nexus_private_t *priv, vhdl_scope *scope,
                                    const string &name)
{
   scope_nexus_t new_sn = { scope, NULL, 0, name, list<ivl_signal_t>() };
   priv->signals.push_back(new_sn);
}

/*
 * Finds the name of the nexus signal within this scope.
 */
static string visible_nexus_signal_name(nexus_private_t *priv, const vhdl_scope *scope,
                                        unsigned *pin)
{
   scope_nexus_t *sn = visible_nexus(priv, scope);
   assert(sn);

   *pin = sn->pin;
   return sn->sig ? get_renamed_signal(sn->sig) : sn->tmpname;
}

/*
 * Calculate the signal type of a nexus. This is modified from
 * draw_net_input in tgt-vvp. This also returns the width of
 * the signal(s) connected to the nexus.
 */
static ivl_signal_type_t signal_type_of_nexus(ivl_nexus_t nex, int &width)
{
   ivl_signal_type_t out = IVL_SIT_TRI;
   width = 0;

   for (unsigned idx = 0; idx < ivl_nexus_ptrs(nex); idx += 1) {
	    ivl_signal_type_t stype;
	    ivl_nexus_ptr_t ptr = ivl_nexus_ptr(nex, idx);
	    ivl_signal_t sig = ivl_nexus_ptr_sig(ptr);
	    if (sig == 0)
         continue;

      width = ivl_signal_width(sig);

	    stype = ivl_signal_type(sig);
	    if (stype == IVL_SIT_TRI)
         continue;
	    if (stype == IVL_SIT_NONE)
         continue;
	    out = stype;
   }

   return out;
}

/*
 * Does nexus `nex' carry a real value: a real signal or a real constant
 * on it? (tgt-vvp's data_type_of_nexus test.) A temporary for it must be
 * a VHDL real, and arithmetic on it real arithmetic.
 */
bool nexus_is_real(ivl_nexus_t nex)
{
   for (unsigned i = 0; nex != NULL && i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_signal_t s = ivl_nexus_ptr_sig(p);
      if (s != NULL && ivl_signal_data_type(s) == IVL_VT_REAL)
         return true;
      ivl_net_const_t c = ivl_nexus_ptr_con(p);
      if (c != NULL && ivl_const_type(c) == IVL_VT_REAL)
         return true;
   }
   return false;
}

// Forward decl: per-generate-iteration name suffix (defined below).
static string genvar_unique_suffix(ivl_scope_t scope);

/*
 * An inout port connected to a bit- or part-select (`.pad(bus[2])',
 * `.pad(bus[3:2])') reaches the parent through a part-select tran
 * (IVL_SW_TRAN_VP): side a is the vector, side b the port's net, `part'
 * bits wide at bit offset `off'. Side b becomes an alias of that element
 * or slice,
 *
 *    alias SW<name>_b is bus(2);   /   alias SW<name>_b is bus(3 downto 2);
 *
 * so the port is associated with the vector itself and both directions
 * work (the vector is resolved: net_needs_resolution sees the switch).
 * Returns NULL when the vector cannot be the actual of an inout port -- an
 * `in' or `out' port of the enclosing entity -- and the caller falls back to
 * a temporary fed by a one-way copy (draw_one_switch).
 */
static vhdl_decl *tran_vp_alias(vhdl_scope *scope, ivl_switch_t sw,
                                const string &name, const vhdl_type *type);

/*
 * A core temporary for a part of a vector: an instance array on a part-select
 * (`pad pa[1:0] (.p(bus[2:1]))') joins each element's port to a part of a
 * temporary, and the temporary to that part of the vector, by part-select
 * trans. If `nex' is such a temporary of scope `sc' -- core temporaries and
 * part-select trans only on it -- return the tran that joins it to the
 * vector (whose side b it is), else NULL.
 */
static ivl_switch_t vp_temp_outer(ivl_nexus_t nex, ivl_scope_t sc)
{
   ivl_switch_t outer = NULL;
   bool temp = false;
   for (unsigned i = 0; nex != NULL && i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_signal_t s = ivl_nexus_ptr_sig(p);
      ivl_switch_t w = ivl_nexus_ptr_switch(p);
      if (s != NULL) {
         if (!ivl_signal_local(s) || ivl_signal_scope(s) != sc)
            return NULL;
         temp = true;
      }
      else if (w != NULL) {
         if (ivl_switch_type(w) != IVL_SW_TRAN_VP || ivl_switch_scope(w) != sc)
            return NULL;
         if (ivl_switch_b(w) == nex) {
            if (outer != NULL)
               return NULL;
            outer = w;
         }
         else if (ivl_switch_a(w) != nex)
            return NULL;
      }
      else
         return NULL;
   }
   return temp ? outer : NULL;
}

// The vector side b of part-select tran `sw' is part of, and the part's
// offset in it: side a, or (vp_temp_outer) the vector a core temporary on
// side a is part of, at the summed offset
static ivl_nexus_t tran_vp_vector(ivl_switch_t sw, unsigned &off)
{
   ivl_nexus_t an = ivl_switch_a(sw);
   off = ivl_switch_offset(sw);
   for (int depth = 0; depth < 8; depth++) {
      ivl_switch_t outer = vp_temp_outer(an, ivl_switch_scope(sw));
      if (outer == NULL)
         break;
      off += ivl_switch_offset(outer);
      an = ivl_switch_a(outer);
   }
   return an;
}

static vhdl_decl *tran_vp_alias(vhdl_scope *scope, ivl_switch_t sw,
                                const string &name, const vhdl_type *type)
{
   // Side a may be a core temporary for a part of a vector: then alias the
   // vector itself (aliasing the temporary would join the port to a one-way
   // copy of the vector)
   unsigned off;
   vhdl_var_ref *a = nexus_to_var_ref(scope, tran_vp_vector(sw, off));
   const unsigned part = ivl_switch_part(sw);
   vhdl_decl *adecl = scope->get_decl(a->get_name());
   const vhdl_type *at = adecl ? adecl->get_type() : NULL;
   const vhdl_port_decl *pd = dynamic_cast<const vhdl_port_decl*>(adecl);
   const char *why = NULL;
   if (at == NULL || at->get_name() != VHDL_TYPE_LOGIC3D_VECTOR)
      why = "the vector is not a logic3d_vector signal";
   else if (pd != NULL && pd->get_mode() != VHDL_PORT_INOUT)
      why = "the vector is an input or output port of the enclosing module";
   if (why != NULL) {
      cerr << "Warning: inout port on " << a->get_name() << "(";
      if (part > 1)
         cerr << off + part - 1 << " downto ";
      cerr << off << ") at " << ivl_switch_file(sw) << ":"
           << ivl_switch_lineno(sw) << " is connected one way only: " << why
           << endl;
      return NULL;
   }
   vhdl_alias_decl *al = new vhdl_alias_decl(name, type, a->get_name(),
                                             off, part);
   ostringstream ss;
   ss << "Inout part-select connection at " << ivl_switch_file(sw) << ":"
      << ivl_switch_lineno(sw);
   al->set_comment(ss.str());
   return al;
}

/*
 * An inout port on a concatenation (`.y({p, q, r, s})'): the core joins the
 * port's net, part by part, to the operands by part-select trans in the
 * scope that instantiates the module, with no net of that scope for the
 * whole. If `nex' is such a port net, fill `parts' with those trans (in
 * offset order; together they cover the port exactly once), `inst' with the
 * instance, and return true. Nothing else may be on the net but the
 * instance's own (its ports, drivers and loads).
 */
static bool concat_port_parts(ivl_nexus_t nex, vector<ivl_switch_t> &parts,
                              ivl_scope_t &inst)
{
   parts.clear();
   inst = NULL;
   ivl_scope_t sc = NULL;
   for (unsigned i = 0; nex != NULL && i < ivl_nexus_ptrs(nex); i++) {
      ivl_switch_t w = ivl_nexus_ptr_switch(ivl_nexus_ptr(nex, i));
      if (w == NULL)
         continue;
      if (ivl_switch_type(w) != IVL_SW_TRAN_VP || ivl_switch_a(w) != nex
          || (sc != NULL && ivl_switch_scope(w) != sc))
         return false;
      sc = ivl_switch_scope(w);
      parts.push_back(w);
   }
   if (parts.empty())
      return false;
   unsigned width = 0;
   for (unsigned i = 0; i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_signal_t s = ivl_nexus_ptr_sig(p);
      ivl_net_logic_t l = ivl_nexus_ptr_log(p);
      ivl_lpm_t m = ivl_nexus_ptr_lpm(p);
      ivl_net_const_t c = ivl_nexus_ptr_con(p);
      ivl_scope_t ps = s ? ivl_signal_scope(s) : l ? ivl_logic_scope(l)
         : m ? ivl_lpm_scope(m) : c ? ivl_const_scope(c) : NULL;
      if (ps == NULL)
         continue;                       // (the switches)
      // the instance of sc this belongs to
      ivl_scope_t in = ps;
      while (in != NULL && ivl_scope_parent(in) != sc)
         in = ivl_scope_parent(in);
      if (in == NULL || ivl_scope_type(in) != IVL_SCT_MODULE
          || (inst != NULL && in != inst))
         return false;
      inst = in;
      if (s != NULL && ivl_signal_scope(s) == inst
          && ivl_signal_port(s) != IVL_SIP_NONE)
         width = ivl_signal_width(s);
   }
   if (inst == NULL || width == 0)
      return false;
   // in offset order, covering [0, width) exactly once
   for (unsigned i = 1; i < parts.size(); i++)
      for (unsigned j = i; j > 0 && ivl_switch_offset(parts[j])
              < ivl_switch_offset(parts[j - 1]); j--)
         std::swap(parts[j], parts[j - 1]);
   unsigned next = 0;
   for (unsigned i = 0; i < parts.size(); i++) {
      if (ivl_switch_offset(parts[i]) != next)
         return false;
      next += ivl_switch_part(parts[i]);
   }
   return next == width;
}

// Is part-select tran `sw' a part of a concatenation on an inout port
// (concat_port_parts)? map_signal associates it; draw_one_switch draws none.
static bool concat_port_part(ivl_switch_t sw)
{
   vector<ivl_switch_t> parts;
   ivl_scope_t inst;
   return ivl_switch_type(sw) == IVL_SW_TRAN_VP
      && concat_port_parts(ivl_switch_a(sw), parts, inst);
}

/*
 * A concatenation operand that is itself a select (`.y({bus[1:0], w})'):
 * the part's side b is then a core temporary that another part-select tran
 * `sel' joins to the vector. Return `sel', or NULL.
 */
static ivl_switch_t concat_operand_select(ivl_switch_t sw)
{
   const ivl_nexus_t b = ivl_switch_b(sw);
   ivl_switch_t sel = NULL;
   for (unsigned i = 0; i < ivl_nexus_ptrs(b); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(b, i);
      ivl_signal_t s = ivl_nexus_ptr_sig(p);
      ivl_switch_t w = ivl_nexus_ptr_switch(p);
      if (s != NULL) {
         if (!ivl_signal_local(s) || ivl_signal_scope(s) != ivl_switch_scope(sw))
            return NULL;
      }
      else if (w != NULL) {
         if (w == sw)
            continue;
         if (sel != NULL || ivl_switch_type(w) != IVL_SW_TRAN_VP
             || ivl_switch_b(w) != b || ivl_switch_scope(w) != ivl_switch_scope(sw))
            return NULL;
         sel = w;
      }
      else
         return NULL;
   }
   return sel;
}

// Is part-select tran `sel' the select of a concatenation operand whose part
// the port association joins to the vector directly (concat_operand_select)?
static bool concat_operand_select_of(ivl_switch_t sel)
{
   const ivl_nexus_t b = ivl_switch_b(sel);
   for (unsigned i = 0; i < ivl_nexus_ptrs(b); i++) {
      ivl_switch_t w = ivl_nexus_ptr_switch(ivl_nexus_ptr(b, i));
      if (w != NULL && w != sel && concat_port_part(w)
          && concat_operand_select(w) == sel)
         return true;
   }
   return false;
}

// draw_one_switch draws nothing for part-select tran `sw': a part of a
// concatenation on an inout port, or the select of such a part's operand
// (the port association joins them, map_signal)
bool tran_vp_drawn_by_port_map(ivl_switch_t sw)
{
   return ivl_switch_type(sw) == IVL_SW_TRAN_VP
      && (concat_port_part(sw) || concat_operand_select_of(sw));
}

/*
 * draw_one_switch draws part-select tran `sw' as a one-way copy of the
 * vector into its side b: does that need the "connected one way only"
 * warning? Not when tran_vp_alias already gave it (side b is then its
 * fallback temporary), nor when nothing uses the copy: a core temporary of
 * an instance array whose every part tran_vp_alias aliased to the vector.
 */
bool tran_vp_copy_needs_warning(vhdl_scope *sc, ivl_switch_t sw)
{
   const ivl_nexus_t b = ivl_switch_b(sw);
   bool own = false;
   for (unsigned i = 0; i < ivl_nexus_ptrs(b); i++) {
      ivl_signal_t s = ivl_nexus_ptr_sig(ivl_nexus_ptr(b, i));
      if (s != NULL && ivl_signal_scope(s) == ivl_switch_scope(sw))
         own = true;
   }
   if (!own)
      return false;
   if (vp_temp_outer(b, ivl_switch_scope(sw)) != sw)
      return true;
   for (unsigned i = 0; i < ivl_nexus_ptrs(b); i++) {
      ivl_switch_t w = ivl_nexus_ptr_switch(ivl_nexus_ptr(b, i));
      if (w == NULL || w == sw || ivl_switch_a(w) != b)
         continue;
      vhdl_var_ref *r = nexus_to_var_ref(sc, ivl_switch_b(w));
      if (dynamic_cast<vhdl_alias_decl*>(sc->get_decl(r->get_name())) == NULL)
         return true;
   }
   return false;
}

// Below: the pull of a tri1/tri0 net, on a signal of an architecture
static void emit_tri_pull(vhdl_arch *arch, vhdl_var_ref *ref, char pull);

/*
 * Generates VHDL code to fully represent a nexus.
 */
void draw_nexus(ivl_nexus_t nexus)
{
   nexus_private_t *priv = new nexus_private_t;
   int nexus_signal_width = -1;
   priv->const_driver = NULL;

   int nptrs = ivl_nexus_ptrs(nexus);

   // Number of drivers for this nexus, of them joins (switch endpoints and
   // inout ports) that drive only what reaches them from elsewhere
   int ndrivers = 0, npassive = 0;

   // A T2 alias (tran_vp_alias) standing for this net in an architecture:
   // where a tri1/tri0 pull of it goes
   vhdl_arch *alias_arch = NULL;
   vhdl_type *alias_type = NULL;
   string alias_name;

   // First pass through connect all the signals up
   for (int i = 0; i < nptrs; i++) {
      ivl_nexus_ptr_t nexus_ptr = ivl_nexus_ptr(nexus, i);

      ivl_signal_t sig;
      if ((sig = ivl_nexus_ptr_sig(nexus_ptr))) {
         vhdl_scope *scope = find_scope_for_signal(sig);
         if (scope) {
            unsigned pin = ivl_nexus_ptr_pin(nexus_ptr);
            link_scope_to_nexus_signal(priv, scope, sig, pin);
         }

         nexus_signal_width = ivl_signal_width(sig);

         // Count output/inout/buffer ports as drivers (an inout one is a
         // join, like a switch endpoint: npassive)
         if (ivl_signal_port(sig) == IVL_SIP_OUTPUT
             || ivl_signal_port(sig) == IVL_SIP_INOUT)
            ndrivers++;
         if (ivl_signal_port(sig) == IVL_SIP_INOUT)
            npassive++;

         // An inout endpoint makes this a bidirectional net: signals sharing
         // it are a true wire join (e.g. b,c shorted through module id(a,a)),
         // which must be modelled as sv_alias, not a one-directional assign.
         // Record the inout's module type so the alias can be marked up.
         if (ivl_signal_port(sig) == IVL_SIP_INOUT) {
            priv->has_inout = true;
            ivl_scope_t ssc = ivl_signal_scope(sig);
            if (ssc != 0)
               priv->inout_module = ivl_scope_tname(ssc);
         }
      }
   }

   // Second pass through make sure logic/LPMs have signal
   // inputs and outputs
   for (int i = 0; i < nptrs; i++) {
      ivl_nexus_ptr_t nexus_ptr = ivl_nexus_ptr(nexus, i);

      ivl_net_logic_t log;
      ivl_lpm_t lpm;
      ivl_net_const_t con;
      if ((log = ivl_nexus_ptr_log(nexus_ptr))) {
         ivl_scope_t log_scope = ivl_logic_scope(log);
         if (!is_default_scope_instance(log_scope))
            continue;

         // An input port buffer is drawn in the parent (map_buffered_input):
         // no temporary for its input in the module
         if (is_input_port_buffer(log)) {
            if (ivl_logic_pin(log, 0) == nexus)
               ndrivers++;
            continue;
         }

         vhdl_entity *ent = find_entity(log_scope);
         assert(ent);

         vhdl_scope *vhdl_scope = ent->get_arch()->get_scope();
         if (visible_nexus(priv, vhdl_scope)) {
            // Already seen this signal in vhdl_scope
         }
         else {
            // Create a temporary signal to connect it to the nexus
            const vhdl_type *type =
               vhdl_type::type_for(ivl_logic_width(log), false);

            ostringstream ss;
            // Append the generate-iteration suffix so the temp net for a
            // logic gate replicated across generate iterations gets a unique
            // name per iteration (matching declare_one_signal).  Without this,
            // every iteration's gate drives the SAME "LO_ivl_N" net -> multiple
            // drivers -> wrong resolution (e.g. VeeR BTB write-enable nets).
            ss << "LO" << ivl_logic_basename(log)
               << genvar_unique_suffix(log_scope);
            // Skip if a signal with this name was already declared
            // (a different nexus connected to the same logic gate)
            if (!vhdl_scope->have_declared(ss.str()))
               vhdl_scope->add_decl(new vhdl_signal_decl(ss.str(), type));

            link_scope_to_nexus_tmp(priv, vhdl_scope, ss.str());
         }

         // If this is connected to pin-0 then this nexus is driven
         // by this logic gate
         if (ivl_logic_pin(log, 0) == nexus)
            ndrivers++;
      }
      else if ((lpm = ivl_nexus_ptr_lpm(nexus_ptr))) {
         ivl_scope_t lpm_scope = ivl_lpm_scope(lpm);
         vhdl_entity *ent = find_entity(lpm_scope);
         if (!ent) continue;  // Skip LPMs in scopes without a VHDL entity (e.g. generate)

         vhdl_scope *vhdl_scope = ent->get_arch()->get_scope();
         if (visible_nexus(priv, vhdl_scope)) {
            // Already seen this signal in vhdl_scope
         }
         else {
            // Create a temporary signal to connect the nexus
            // TODO: we could avoid this for IVL_LPM_PART_PV

            // If we already know how wide the temporary should be
            // (i.e. because we've seen a signal it's connected to)
            // then use that, otherwise use the width of the LPM
            int lpm_temp_width;
            if (nexus_signal_width != -1)
               lpm_temp_width = nexus_signal_width;
            else
               lpm_temp_width = ivl_lpm_width(lpm);

            // A real net (a real port's actual `r1 + 0.2', an operand of
            // real arithmetic) needs a real temporary: a logic3d one
            // would carry the value through real_to_l3d1, i.e. lose it
            const vhdl_type *type = nexus_is_real(nexus)
               ? vhdl_type::real()
               : vhdl_type::type_for(lpm_temp_width, ivl_lpm_signed(lpm) != 0);
            ostringstream ss;
            ss << "LPM";
            if (nexus == ivl_lpm_q(lpm))
               ss << "_q";
            else {
               // For IVL_LPM_REPEAT, ivl_lpm_size is the repeat count but
               // only data(0) is a valid input — iterating with d>0 trips
               // an assertion. Cap at 1 for REPEAT.
               unsigned ndata = (ivl_lpm_type(lpm) == IVL_LPM_REPEAT)
                              ? 1 : ivl_lpm_size(lpm);
               for (unsigned d = 0; d < ndata; d++) {
                  if (nexus == ivl_lpm_data(lpm, d))
                     ss << "_d" << d;
               }
            }
            ss << ivl_lpm_basename(lpm);
            // Append the generate-iteration suffix so an LPM temp net (e.g. a
            // synthesized FF q/d or enable) replicated across generate
            // iterations gets a unique name per iteration — mirroring the logic
            // gate ("LO") case above.  Without this, every iteration's LPM
            // shares the SAME "LPM_*_ivl_N" net, so distinct nets (a 1-bit FF
            // enable and a wide FF din) alias to one signal -> width/type
            // mismatch + multiple drivers (VeeR EH2 IC_TAG/BTB write-bypass).
            ss << genvar_unique_suffix(lpm_scope);

            // The genvar suffix disambiguates LPM temps replicated across
            // generate iterations, but synthesized FF q-nets can be re-parented
            // to a non-generate scope (empty suffix) and still collide — two
            // DISTINCT nets sharing one basename.  We only reach here when this
            // nexus is NEW to the scope (visible_nexus was false above), so a
            // name already declared belongs to a DIFFERENT nexus: uniquify it
            // rather than alias both nets onto one signal (which produced the
            // 1-bit-enable vs wide-din width clashes in VeeR EH2 IC_TAG/BTB).
            string tname = ss.str();
            if (vhdl_scope->have_declared(tname)) {
               ostringstream uq;
               int u = 1;
               do { uq.str(""); uq << ss.str() << "_u" << u++; }
               while (vhdl_scope->have_declared(uq.str()));
               tname = uq.str();
            }
            vhdl_scope->add_decl(new vhdl_signal_decl(tname, type));

            link_scope_to_nexus_tmp(priv, vhdl_scope, tname);
         }

         // If this is connected to the LPM output then this nexus
         // is driven by the LPM
         if (ivl_lpm_q(lpm) == nexus)
            ndrivers++;
      }
      else if ((con = ivl_nexus_ptr_con(nexus_ptr))) {
         if (ivl_const_type(con) == IVL_VT_REAL) {
            priv->const_driver = new vhdl_const_real(ivl_const_real(con));
            priv->const_drive0 = ivl_nexus_ptr_drive0(nexus_ptr);
            priv->const_drive1 = ivl_nexus_ptr_drive1(nexus_ptr);
            ndrivers++;
            continue;
         }
         vhdl_expr *cexpr;
         if (ivl_const_width(con) == 1)
            cexpr = new vhdl_const_bit(ivl_const_bits(con)[0]);
         else
            cexpr =
               new vhdl_const_bits(ivl_const_bits(con), ivl_const_width(con),
                                   ivl_const_signed(con) != 0);
         const string cbits(ivl_const_bits(con), ivl_const_width(con));

         if (priv->const_driver == NULL) {
            priv->const_driver = cexpr;
            priv->const_drive0 = ivl_nexus_ptr_drive0(nexus_ptr);
            priv->const_drive1 = ivl_nexus_ptr_drive1(nexus_ptr);
            priv->const_bits = cbits;
         }
         else {
            const_drv_t extra = { cexpr, ivl_nexus_ptr_drive0(nexus_ptr),
                                  ivl_nexus_ptr_drive1(nexus_ptr), cbits };
            priv->const_extra.push_back(extra);
         }

         // A constant is a sort of driver
         ndrivers++;
      }
      else {
         // Check for switch connections (tran, tranif, etc.)
         ivl_switch_t sw = ivl_nexus_ptr_switch(nexus_ptr);
         if (sw) {
            ivl_scope_t sw_scope = ivl_switch_scope(sw);
            if (!is_default_scope_instance(sw_scope))
               continue;

            vhdl_entity *ent = find_entity(sw_scope);
            if (!ent) continue;

            vhdl_scope *vhdl_scope = ent->get_arch()->get_scope();
            if (!visible_nexus(priv, vhdl_scope)) {
               // Create a temporary for this switch connection. In sv2vhdl
               // mode nets are logic3d, and this temp is bound to a logic3d
               // inout port, so it must be logic3d too (not std_logic); the
               // narrow side of a part-select tran is as wide as the part.
               const bool part_b = get_sv2vhdl_mode()
                  && ivl_switch_type(sw) == IVL_SW_TRAN_VP
                  && nexus == ivl_switch_b(sw);
               const unsigned width = part_b ? ivl_switch_part(sw) : 1;
               const vhdl_type *type = !get_sv2vhdl_mode()
                  ? vhdl_type::std_logic()
                  : width > 1 ? vhdl_type::logic3d_vector(width - 1, 0)
                              : vhdl_type::logic3d();

               // SW<switch><genvar suffix>_a|_b|_en: the genvar suffix keeps
               // the copies of one switch replicated by a generate loop
               // apart (they share a basename), and a name declared for
               // another nexus is never reused (_u<k>), since sharing it
               // would short two nets together.
               const char *side = nexus == ivl_switch_a(sw) ? "_a"
                  : nexus == ivl_switch_b(sw) ? "_b" : "_en";
               const string stem = string("SW") + ivl_switch_basename(sw)
                  + genvar_unique_suffix(sw_scope);
               string tname = stem + side;
               for (int u = 1; vhdl_scope->have_declared(tname); u++) {
                  ostringstream uq;
                  uq << stem << "_u" << u << side;
                  tname = uq.str();
               }

               vhdl_decl *decl = part_b
                  ? tran_vp_alias(vhdl_scope, sw, tname, type) : NULL;
               if (decl != NULL) {
                  // (an alias keeps its target's index range)
                  unsigned aoff;
                  tran_vp_vector(sw, aoff);
                  alias_arch = ent->get_arch();
                  alias_name = tname;
                  alias_type = width > 1
                     ? vhdl_type::logic3d_vector(aoff + width - 1, aoff)
                     : vhdl_type::logic3d();
               }
               if (decl == NULL)
                  decl = new vhdl_signal_decl(tname, type);
               vhdl_scope->add_decl(decl);

               link_scope_to_nexus_tmp(priv, vhdl_scope, tname);
            }

            // Switch ports are bidirectional drivers
            if (ivl_switch_a(sw) == nexus || ivl_switch_b(sw) == nexus) {
               ndrivers++;
               npassive++;
            }
         }
      }
   }

   // sv2vhdl mode: a tri1/tri0 net is pulled whether or not something else
   // drives it (draw_constant_drivers places the pull)
   if (get_sv2vhdl_mode()) {
      int width;
      const ivl_signal_type_t st = signal_type_of_nexus(nexus, width);
      if (st == IVL_SIT_TRI1 || st == IVL_SIT_TRI0) {
         priv->pull = st == IVL_SIT_TRI1 ? '1' : '0';
         // The net of a (coerced) inout port on a bit- or part-select: its
         // T2 alias stands for it in the parent, and carries the pull
         if (alias_arch != NULL) {
            emit_tri_pull(alias_arch, new vhdl_var_ref(alias_name, alias_type),
                          priv->pull);
            priv->pull = 0;
         }
         else
            g_pull_nexus.push_back(nexus);
      }
   }

   // Drive undriven nets with a constant. An undriven real net reads 0.0
   // (as in vvp): a logic3d Z would not even be of its type; 0.0 is also
   // what an unconnected real input port is given (map_signal). A real
   // variable is no net: it keeps the values its processes give it.
   // A net that only joins drive -- switches and inout ports, i.e. nothing
   // but what reaches it from elsewhere -- gets its default too (in sv2vhdl
   // mode such a net is resolved, e.g. the vector of a T2 alias): resolved
   // away wherever a value arrives, and a bit none drives reads Z, not X.
   int real_width;
   if (ndrivers == 0 && nexus_is_real(nexus)) {
      const ivl_signal_type_t st = signal_type_of_nexus(nexus, real_width);
      if (st == IVL_SIT_TRI || st == IVL_SIT_UWIRE)
         priv->const_driver = new vhdl_const_real(0.0);
   }
   else if (ndrivers == 0
            || (get_sv2vhdl_mode() && ndrivers == npassive
                && priv->const_driver == NULL && !nexus_is_real(nexus))) {
      // (a joined net's tri0/tri1 value is its pull's, not a constant's)
      const bool joined = ndrivers != 0;
      char def = 0;
      int width;

      switch (signal_type_of_nexus(nexus, width)) {
      case IVL_SIT_TRI:
      case IVL_SIT_UWIRE:
         def = 'Z';
         break;
      case IVL_SIT_TRI0:
         def = joined ? 0 : '0';
         break;
      case IVL_SIT_TRI1:
         def = joined ? 0 : '1';
         break;
      case IVL_SIT_TRIAND:
         if (!joined)
            error("No VHDL translation for triand nets");
         break;
      case IVL_SIT_TRIOR:
         if (!joined)
            error("No VHDL translation for trior nets");
         break;
      default:
         ;
      }

      if (def) {
         if (width > 1)
            priv->const_driver =
               new vhdl_bit_spec_expr(vhdl_type::std_logic(),
                                      new vhdl_const_bit(def));
         else
            priv->const_driver = new vhdl_const_bit(def);
         priv->tri_default = priv->pull != 0;
      }
   }

   // Save the private data in the nexus
   ivl_nexus_set_private(nexus, priv);
}

/*
 * Draw the pull of a tri1/tri0 net on `ref' (a signal of the architecture):
 * one sv2vhdl.sv_pullup / sv_pulldown per bit, each marked with the pull
 * strength the cut analysis reads (-- sv_strength: pull1 pull0).
 */
static void emit_tri_pull(vhdl_arch *arch, vhdl_var_ref *ref, char pull)
{
   const char *ent = pull == '1' ? "sv_pullup" : "sv_pulldown";
   const vhdl_type *t = ref->get_type();
   const bool vec = t != NULL && t->get_name() == VHDL_TYPE_LOGIC3D_VECTOR;
   const int lo = vec ? t->get_lsb() : 0;
   const int hi = vec ? t->get_msb() : 0;
   for (int b = lo; b <= hi; b++) {
      ostringstream lbl;
      lbl << "sv_tri" << pull << "_" << ref->get_name();
      if (vec)
         lbl << "_b" << b;
      string label = lbl.str();
      for (int u = 1; arch->get_scope()->have_declared(label); u++) {
         ostringstream uq;
         uq << lbl.str() << "_u" << u;
         label = uq.str();
      }
      vhdl_entity_inst *inst = new vhdl_entity_inst(
         label.c_str(), "sv2vhdl", ent, "behavioral");
      vhdl_var_ref *y = new vhdl_var_ref(ref->get_name(),
         vec ? vhdl_type::logic3d() : new vhdl_type(*t));
      if (vec)
         y->set_slice(new vhdl_const_int(b));
      inst->map_port("y", y);
      inst->set_comment("sv_strength: pull1 pull0");
      arch->add_stmt(inst);
   }
}

/*
 * A tri1/tri0 pull that found no plain signal to sit on -- the net is made
 * of ports only, e.g. a tri1 port of the root module, or a tri1 output bound
 * to an expression -- is not modelled: say so. (An unconnected tri1/tri0
 * input takes the constant in its port map instead; see map_signal.)
 */
void report_unplaced_pulls()
{
   for (list<ivl_nexus_t>::const_iterator it = g_pull_nexus.begin();
        it != g_pull_nexus.end(); ++it) {
      nexus_private_t *priv =
         static_cast<nexus_private_t*>(ivl_nexus_get_private(*it));
      if (priv == NULL || priv->pull == 0)
         continue;
      const char *name = "?";
      for (unsigned i = 0; i < ivl_nexus_ptrs(*it); i++) {
         ivl_signal_t s = ivl_nexus_ptr_sig(ivl_nexus_ptr(*it, i));
         if (s != NULL && (ivl_signal_type(s) == IVL_SIT_TRI1
                           || ivl_signal_type(s) == IVL_SIT_TRI0)) {
            name = ivl_signal_name(s);
            break;
         }
      }
      cerr << "Warning: tri" << priv->pull << " net " << name
           << ": its pull is not translated (no internal signal to attach "
              "it to)" << endl;
   }
   g_pull_nexus.clear();
}

/*
 * Ensure that a nexus has been initialised. I.e. all the necessary
 * statements, declarations, etc. have been generated.
 */
void seen_nexus(ivl_nexus_t nexus)
{
   if (ivl_nexus_get_private(nexus) == NULL)
      draw_nexus(nexus);
}

/*
 * Translate a nexus to a variable reference. Given a nexus and a
 * scope, this function returns a reference to a signal that is
 * connected to the nexus and within the given scope. This signal
 * might not exist in the original Verilog source (even as a
 * compiler-generated temporary). If this nexus hasn't been
 * encountered before, the necessary code to connect up the nexus
 * will be generated.
 */
// Non-asserting probe: is some signal on this nexus visible in scope?
// (nexus_to_var_ref asserts on failure; icg2en needs a soft test)
bool nexus_visible_in_scope(vhdl_scope *scope, ivl_nexus_t nexus)
{
   seen_nexus(nexus);
   nexus_private_t *priv =
      static_cast<nexus_private_t*>(ivl_nexus_get_private(nexus));
   if (priv == NULL)
      return false;
   return visible_nexus(priv, scope) != NULL;
}

vhdl_var_ref *nexus_to_var_ref(vhdl_scope *scope, ivl_nexus_t nexus)
{
   seen_nexus(nexus);

   nexus_private_t *priv =
      static_cast<nexus_private_t*>(ivl_nexus_get_private(nexus));
   unsigned pin;
   string renamed(visible_nexus_signal_name(priv, scope, &pin));

   vhdl_decl *decl = scope->get_decl(renamed);
   assert(decl);

   const vhdl_type *type = new vhdl_type(*(decl->get_type()));
   vhdl_var_ref *ref = new vhdl_var_ref(renamed, type);

   if (decl->get_type()->get_name() == VHDL_TYPE_ARRAY)
      ref->set_slice(new vhdl_const_int(pin), 0);

   return ref;
}

// Return a variable reference for a nexus that is guaranteed to
// be readable.
vhdl_var_ref* readable_ref(vhdl_scope* scope, ivl_nexus_t nex)
{
   vhdl_var_ref* ref = nexus_to_var_ref(scope, nex);

   vhdl_decl* decl = scope->get_decl(ref->get_name());
   decl->ensure_readable();

   return ref;
}

/*
 * Translate all the primitive logic gates into concurrent
 * signal assignments.
 */
static void declare_logic(vhdl_arch *arch, ivl_scope_t scope)
{
   debug_msg("Declaring logic in scope type %s", ivl_scope_tname(scope));

   int nlogs = ivl_scope_logs(scope);
   for (int i = 0; i < nlogs; i++)
      draw_logic(arch, ivl_scope_log(scope, i));
}

// Replace consecutive underscores with a single underscore
void replace_consecutive_underscores(string& str)
{
   size_t pos = str.find("__");
   while (pos != string::npos) {
      str.replace(pos, 2, "_");
      pos = str.find("__");
   }
}

bool is_vhdl_reserved_word(const string& word)
{
   // Reserved words that must never be emitted as a bare identifier.
   // The generated VHDL is analysed at the highest standard the target
   // simulator supports, so every word reserved by ANY revision (plus the
   // simulator's own extensions) has to be renamed.
   const char *vhdl_reserved[] = {
      // IEEE 1076-1993 (section 13.9)
      "abs", "access", "after", "alias", "all", "and", "architecture",
      "array", "assert", "attribute", "begin", "block", "body", "buffer",
      "bus", "case", "component", "configuration", "constant", "disconnect",
      "downto", "else", "elsif", "end", "entity", "exit", "file", "for",
      "function", "generate", "generic", "group", "guarded", "if", "impure",
      "in", "inertial", "inout", "is", "label", "library", "linkage",
      "literal", "loop", "map", "mod", "nand", "new", "next", "nor", "not",
      "null", "of", "on", "open", "or", "others", "out", "package", "port",
      "postponed", "procedure", "process", "pure", "range", "record",
      "register", "reject", "rem", "report", "return", "rol", "ror", "select",
      "severity", "signal", "shared", "sla", "sll", "sra", "srl", "subtype",
      "then", "to", "transport", "type", "unaffected", "units", "until", "use",
      "variable", "wait", "when", "while", "with", "xnor", "xor",
      // IEEE 1076-2000/2002
      "protected",
      // IEEE 1076-2008 (section 15.10): keywords and PSL reserved words
      "assume", "assume_guarantee", "context", "cover", "default",
      "fairness", "force", "parameter", "property", "release", "restrict",
      "restrict_guarantee", "sequence", "strong", "vmode", "vprop", "vunit",
      // IEEE 1076-2019
      "private", "view", "vpkg",
      // NVC: tokenised unconditionally in every standard (src/lexer.l)
      "reverse_range",
      // NVC fork (kev-cam) --std=2040 pipe construct (PIPES.md); the lexer
      // rule is not gated on the standard so it is reserved in all modes
      "pipe",
      NULL
   };

   for (const char **p = vhdl_reserved; *p != NULL; p++) {
      if (strcasecmp(*p, word.c_str()) == 0)
         return true;
   }

   return false;
}

// Return a valid VHDL name for a Verilog module
static string valid_entity_name(const string& module_name)
{
   string name(module_name);
   replace_consecutive_underscores(name);
   if (name[0] == '_')
      name = "module" + name;
   if (*name.rbegin() == '_')
      name += "module";

   if (is_vhdl_reserved_word(name))
      name += "_module";

   ostringstream ss;
   int i = 1;
   ss << name;
   while (find_entity(ss.str())) {
      // Keep adding an extra number until we get a unique name
      ss.str("");
      ss << name << i++;
   }

   return ss.str();
}

// Make sure a signal name conforms to VHDL naming rules.
string make_safe_name(ivl_signal_t sig)
{
   string base(ivl_signal_basename(sig));

   if (ivl_signal_local(sig))
      base = "tmp" + base;

   if (base[0] == '_')
      base = "sig" + base;

   if (*base.rbegin() == '_')
      base += "sig";

   // Can't have two consecutive underscores
   replace_consecutive_underscores(base);

   // A signal name may not be the same as a component name
   if (find_entity(base) != NULL)
      base += "_sig";

   if (is_vhdl_reserved_word(base))
      base += "_sig";

   return base;
}

// Check if `name' differs from an existing name only in case and
// make it unique if it does.
static void avoid_name_collision(string& name, const vhdl_scope* scope)
{
   if (scope->name_collides(name)) {
      name += "_";
      ostringstream ss;
      int i = 1;
      do {
         // Keep adding an extra number until we get a unique name
         ss.str("");
         ss << name << i++;
      } while (scope->name_collides(ss.str()));

      name = ss.str();
   }
}

// Make a suffix that keeps the contents of one generate scope apart from
// every other generate scope of the containing module, since generate
// blocks are flattened into the enclosing architecture.
// This isn't ideal: it would be better to replace the Verilog
// generate with an equivalent VHDL generate, but this isn't possible
// with the current API
//
// The suffix is the path of generate-block names from the module down to
// `scope', `name[idx]' loop iterations rendered as `name_idx'. Verilog
// scope names are unique within their parent, so the path is unique by
// construction and stays short. (Earlier versions folded the numeric
// parameters of loop-iteration scopes in instead: two sibling conditional
// blocks -- e.g. two interface-port modules that sv2v inlined as named
// generate blocks -- got the same empty suffix, and two loops with equal
// genvar values under different siblings collided as well, so `rsp_buf'
// in alu_unit and in lsu_unit became one label.)
static string genvar_unique_suffix(ivl_scope_t scope)
{
   std::vector<std::string> path;
   while (scope && ivl_scope_type(scope) == IVL_SCT_GENERATE) {
      std::string part(ivl_scope_basename(scope));
      for (std::string::size_type i = 0; i < part.size(); i++) {
         if (part[i] == '[')
            part[i] = '_';
         else if (!isalnum((unsigned char)part[i]) && part[i] != '_')
            part[i] = '\0';
      }
      part.erase(std::remove(part.begin(), part.end(), '\0'), part.end());
      path.push_back(part);
      scope = ivl_scope_parent(scope);
   }

   ostringstream suffix;
   for (std::vector<std::string>::reverse_iterator it = path.rbegin();
        it != path.rend(); ++it)
      suffix << "_" << *it;

   return suffix.str();
}

// Declare a single signal in a scope
// A signal living in a PACKAGE or $unit/compilation-unit scope is never
// visited by the entity walk, so a reference to it (import p1::x) died on
// get_renamed_signal's assertion. Give it a home on first use: declare it in
// the referencing entity's architecture, package-prefixed. Verilog package
// variables are global shared state; a per-referencing-entity copy is an
// approximation that holds for the (dominant) single-module tests --
// cross-module package-variable sharing would need the C-side store.
void ensure_signal_declared(ivl_signal_t sig)
{
   if (seen_signal_before(sig))
      return;

   ivl_scope_t sscope = ivl_signal_scope(sig);
   const ivl_scope_type_t st = ivl_scope_type(sscope);
   // Package/$unit scopes are never walked; named begin/fork and generate
   // scopes can also carry locals the walk missed.
   if (st != IVL_SCT_PACKAGE && st != IVL_SCT_MODULE
       && st != IVL_SCT_BEGIN && st != IVL_SCT_FORK
       && st != IVL_SCT_GENERATE && st != IVL_SCT_TASK
       && st != IVL_SCT_FUNCTION)
      return;

   vhdl_entity *ent = get_active_entity();
   if (NULL == ent)
      return;

   std::string name(ivl_scope_basename(sscope));
   name += "_";
   name += ivl_signal_basename(sig);
   // $unit scope names contain characters VHDL identifiers cannot ($unit#...)
   for (std::string::size_type i = 0; i < name.size(); i++) {
      const char c = name[i];
      if (!isalnum(c) && c != '_')
         name[i] = '_';
   }
   while (!name.empty() && (name[0] == '_' || isdigit(name[0])))
      name.erase(0, 1);
   if (name.empty())
      name = "unit_var";
   // VHDL rejects consecutive underscores in identifiers
   std::string::size_type p;
   while ((p = name.find("__")) != std::string::npos)
      name.erase(p, 1);

   vhdl_scope *ascope = ent->get_arch()->get_scope();
   if (!ascope->have_declared(name)) {
      vhdl_signal_decl *decl =
         new vhdl_signal_decl(name, vhdl_type_for_signal(sig));
      decl->set_comment("Package/unit-scope variable given a local home");

      // A package variable's initializer appears as a constant driver on its
      // nexus (the package scope itself is never walked) -- carry it onto the
      // local declaration or `int x = 5;` in a package silently becomes 0.
      ivl_nexus_t nex = ivl_signal_nex(sig, 0);
      if (nex != NULL) {
         const int nptrs = ivl_nexus_ptrs(nex);
         for (int i = 0; i < nptrs; i++) {
            ivl_net_const_t con =
               ivl_nexus_ptr_con(ivl_nexus_ptr(nex, i));
            if (con == NULL)
               continue;
            if (ivl_const_type(con) == IVL_VT_REAL)
               decl->set_initial(new vhdl_const_real(ivl_const_real(con)));
            else if (ivl_const_width(con) == 1)
               decl->set_initial(new vhdl_const_bit(ivl_const_bits(con)[0]));
            else
               decl->set_initial(new vhdl_const_bits(
                  ivl_const_bits(con), ivl_const_width(con),
                  ivl_const_signed(con) != 0));
            break;
         }
      }
      ascope->add_decl(decl);
   }
   remember_signal(sig, ascope);
   rename_signal(sig, name);
}

vhdl_type *vhdl_type_for_signal(ivl_signal_t sig)
{
   if (ivl_signal_data_type(sig) == IVL_VT_REAL)
      return vhdl_type::real();
   return vhdl_type::type_for(ivl_signal_width(sig),
                              ivl_signal_signed(sig) != 0);
}

// Does anything inside sig's OWN module drive its (inout-port) net? nvc's inout
// ports read their own default rather than the externally-driven value, so a
// read-only inout Verilog port must be declared `in` to see the connected
// value. Only report "driven" for a driver in the port's own scope (external
// and submodule connections live in other scopes); be conservative -- if a net
// join or unknown driver could exist, keep it inout (safe: no regression).
static bool inout_driven_internally(ivl_signal_t sig)
{
   ivl_scope_t sig_scope = ivl_signal_scope(sig);
   ivl_nexus_t nex = ivl_signal_nex(sig, 0);
   if (nex == NULL)
      return true;                       // unknown -> keep inout
   int nptrs = ivl_nexus_ptrs(nex);
   for (int i = 0; i < nptrs; i++) {
      ivl_nexus_ptr_t ptr = ivl_nexus_ptr(nex, i);
      ivl_net_logic_t log = ivl_nexus_ptr_log(ptr);
      if (log && ivl_logic_scope(log) == sig_scope
          && ivl_logic_pin(log, 0) == nex)
         return true;                    // driven by an in-scope gate output
      ivl_lpm_t lpm = ivl_nexus_ptr_lpm(ptr);
      if (lpm && ivl_lpm_scope(lpm) == sig_scope
          && ivl_lpm_q(lpm) == nex)
         return true;                    // driven by an in-scope LPM output
      ivl_net_const_t con = ivl_nexus_ptr_con(ptr);
      if (con)
         return true;                    // a constant driver
      // A switch (tran/tranif/relay) in the port's OWN scope makes it a genuine
      // bidirectional endpoint, and its formal is inout -- keep the port inout
      // so the port-map modes match. (A part-select tran for an instance
      // connection lives in the PARENT scope, so a read-only port there still
      // becomes `in`.) A switch in a generate block of the port's module is
      // in its own scope too.
      ivl_switch_t sw = ivl_nexus_ptr_switch(ptr);
      if (sw) {
         ivl_scope_t ssc = ivl_switch_scope(sw);
         while (ssc != NULL && ivl_scope_type(ssc) == IVL_SCT_GENERATE)
            ssc = ivl_scope_parent(ssc);
         if (ssc == sig_scope)
            return true;
      }
      // Sharing the net with another module's inout port (pass-through short)
      // is also bidirectional.
      ivl_signal_t s2 = ivl_nexus_ptr_sig(ptr);
      if (s2 && s2 != sig && ivl_signal_port(s2) == IVL_SIP_INOUT
          && ivl_signal_scope(s2) != sig_scope)
         return true;
   }
   return false;                         // read-only inside this module
}

// Below: an input port declared inout, its net being driven inside its module
static bool input_driven_inside(ivl_signal_t sig);

// A net needs a resolved logic3d subtype when several sources can drive it, so
// their contributions combine (rather than nvc erroring or one silently
// winning): two or more drivers, a switch/tran endpoint, or a share with an
// inout port (or an input port declared inout, input_driven_inside). This is
// the bidirectional/mixed-signal model -- direction is advisory; the net
// itself is a resolved meeting point.
static bool net_needs_resolution(ivl_signal_t sig)
{
   if (!get_sv2vhdl_mode())
      return false;
   ivl_nexus_t nex = ivl_signal_nex(sig, 0);
   if (nex == NULL)
      return false;
   int nptrs = ivl_nexus_ptrs(nex);
   int drivers = 0;
   bool pulled = false;
   for (int i = 0; i < nptrs; i++) {
      ivl_nexus_ptr_t ptr = ivl_nexus_ptr(nex, i);
      ivl_net_logic_t log = ivl_nexus_ptr_log(ptr);
      if (log && ivl_logic_pin(log, 0) == nex)
         drivers++;
      ivl_lpm_t lpm = ivl_nexus_ptr_lpm(ptr);
      if (lpm && ivl_lpm_q(lpm) == nex)
         drivers++;
      if (ivl_nexus_ptr_con(ptr))
         drivers++;
      if (ivl_nexus_ptr_switch(ptr))
         return true;                    // a tran/relay endpoint
      ivl_signal_t s2 = ivl_nexus_ptr_sig(ptr);
      if (s2 && (ivl_signal_type(s2) == IVL_SIT_TRI1
                 || ivl_signal_type(s2) == IVL_SIT_TRI0))
         pulled = true;
      if (s2 && s2 != sig) {
         ivl_signal_port_t pt = ivl_signal_port(s2);
         if (pt == IVL_SIP_INOUT)
            return true;                 // shares the net with an inout port
         if (pt == IVL_SIP_OUTPUT)
            drivers++;
         if (pt == IVL_SIP_INPUT && ivl_signal_scope(s2) != ivl_signal_scope(sig)
             && ivl_signal_type(sig) != IVL_SIT_REG && input_driven_inside(s2))
            return true;                 // ... with an input declared inout
                                         // (a net: a variable stays one way)
      }
   }
   // A tri1/tri0 net gets a pull instance (emit_tri_pull)
   if (pulled)
      drivers++;
   return drivers >= 2;
}

// Is scope `s' the scope `anc' or inside it?
static bool scope_within(ivl_scope_t s, ivl_scope_t anc)
{
   for (; s != NULL; s = ivl_scope_parent(s))
      if (s == anc)
         return true;
   return false;
}

/*
 * Input port networks.
 *
 * When an input port's net is also driven inside its module (a pull, an
 * assign, a child's output; see input_driven_inside) and the actual cannot
 * join that net -- a variable, an expression, a select or a constant; a net
 * actual coerces the port to inout instead -- the iverilog core buffers the
 * actual: a transparent buffer (IVL_LO_BUFT) in the module instance, its
 * input on the actual, a net of the parent, its output on the port's net or
 * on a core temporary. Between such a temporary and the port the core puts
 * what the connection needs:
 *
 *    - a pad to the port width: IVL_LPM_CONCAT of the value and a constant
 *      zero, or IVL_LPM_SIGN_EXT for a signed value;
 *    - a prune to the port width: IVL_LPM_PART_VP at offset 0;
 *    - for an instance array, the split of the value across the elements:
 *      an IVL_LPM_PART_VP per element.
 *
 * Of an instance array, all of it sits in the scope of element [0], whatever
 * element a part feeds; and a value as wide as one element's port is shared:
 * the elements' ports are one net.
 *
 * This network (the buffer, its nodes and temporaries) is the connection of
 * the instance, so it is drawn in the parent, per instance (map_signal,
 * network_parent_ref), and not in the module:
 *
 *    signal PB_<label>_<port> : resolved_logic3d...;    -- the port's net
 *    PB_<label>_<port> <= <actual>;            (a buffer straight on the port)
 *    PB_<label>_<port> <= <actual>(1 downto 0);         (a prune or a part)
 *    PB_<label>_<port> <= L3D_0 & <actual>;             (a pad)
 *    <label>: entity work.<module> port map (<port> => PB_<label>_<port>, ..
 *
 * (an actual that is no plain signal of the parent, or a pad before a split,
 * goes through a temporary PBT_<label>_<port> first).
 *
 * so the actual drives the port's net and the module's own drivers of that
 * net (the reason for the buffer; the port is then inout) stop at the
 * signal, as in Verilog. It is drawn per instance: another instance of the
 * module may have a net actual, which the core joins to the port's net. A
 * pull of a tri1/tri0 port net goes on its PB_ signal. A buffer the
 * translation cannot draw is an error (untranslated_port_buffer), and so is
 * a port whose connection is lost on the way (map_signal): never a port
 * left silently open.
 */

// The instance-array name of a module instance: "s" for "s[2]"; false for an
// instance that is not an element of an array
static bool array_base_name(ivl_scope_t s, string &base)
{
   const string bn = ivl_scope_basename(s);
   if (bn.empty() || bn[bn.size() - 1] != ']')
      return false;
   const string::size_type br = bn.rfind('[');
   if (br == string::npos || br == 0)
      return false;
   base = bn.substr(0, br);
   return true;
}

// Is scope `s' an instance of the group of module instance `grp': grp
// itself, or another element of the same instance array?
static bool in_instance_group(ivl_scope_t s, ivl_scope_t grp)
{
   if (s == grp)
      return true;
   if (s == NULL || grp == NULL || ivl_scope_type(s) != IVL_SCT_MODULE
       || ivl_scope_type(grp) != IVL_SCT_MODULE
       || ivl_scope_parent(s) != ivl_scope_parent(grp)
       || strcmp(ivl_scope_tname(s), ivl_scope_tname(grp)) != 0)
      return false;
   string a, b;
   return array_base_name(s, a) && array_base_name(grp, b) && a == b;
}

// Is scope `s' one of the instances of the group of `grp', or inside one?
static bool within_instance_group(ivl_scope_t s, ivl_scope_t grp)
{
   for (; s != NULL; s = ivl_scope_parent(s))
      if (in_instance_group(s, grp))
         return true;
   return false;
}

// An input port of the instance group of `grp' on nexus `nex', or NULL
static ivl_signal_t group_input_port(ivl_nexus_t nex, ivl_scope_t grp)
{
   for (unsigned i = 0; nex != NULL && i < ivl_nexus_ptrs(nex); i++) {
      ivl_signal_t s = ivl_nexus_ptr_sig(ivl_nexus_ptr(nex, i));
      if (s != NULL && ivl_signal_port(s) == IVL_SIP_INPUT
          && in_instance_group(ivl_signal_scope(s), grp))
         return s;
   }
   return NULL;
}

// The constant input of a pad (pad_to_width): constants of 0 bits and core
// temporaries on the net, and the pad -- nothing else
static bool zero_pad_nexus(ivl_nexus_t nex, ivl_lpm_t pad)
{
   bool zero = false;
   for (unsigned i = 0; nex != NULL && i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_signal_t s = ivl_nexus_ptr_sig(p);
      ivl_net_const_t c = ivl_nexus_ptr_con(p);
      if (s != NULL) {
         if (!ivl_signal_local(s))
            return false;
      }
      else if (c != NULL) {
         if (ivl_const_type(c) == IVL_VT_REAL)
            return false;
         const char *bits = ivl_const_bits(c);
         for (unsigned b = 0; b < ivl_const_width(c); b++)
            if (bits[b] != '0')
               return false;
         zero = true;
      }
      else if (ivl_nexus_ptr_lpm(p) != pad)
         return false;
   }
   return zero;
}

// A node kind of an input port network (a prune or split: IVL_LPM_PART_VP
// at a constant offset; a pad: IVL_LPM_CONCAT with a constant zero, or
// IVL_LPM_SIGN_EXT); `in' is its value input
static bool network_node_kind(ivl_lpm_t lpm, ivl_nexus_t &in)
{
   in = NULL;
   switch (ivl_lpm_type(lpm)) {
   case IVL_LPM_PART_VP:
      if (ivl_lpm_data(lpm, 1) != NULL)
         return false;
      in = ivl_lpm_data(lpm, 0);
      break;
   case IVL_LPM_SIGN_EXT:
      in = ivl_lpm_data(lpm, 0);
      break;
   case IVL_LPM_CONCAT:
      if (ivl_lpm_size(lpm) != 2 || !zero_pad_nexus(ivl_lpm_data(lpm, 1), lpm))
         return false;
      in = ivl_lpm_data(lpm, 0);
      break;
   default:
      return false;
   }
   return in != NULL;
}

// The input of a buffer comes from outside the instance group: no net of the
// group is on it but its input ports (a port that shares the actual)
static bool from_outside(ivl_nexus_t in, ivl_scope_t grp)
{
   if (in == NULL)
      return false;
   for (unsigned j = 0; j < ivl_nexus_ptrs(in); j++) {
      ivl_signal_t s = ivl_nexus_ptr_sig(ivl_nexus_ptr(in, j));
      if (s != NULL && ivl_signal_port(s) != IVL_SIP_INPUT
          && within_instance_group(ivl_signal_scope(s), grp))
         return false;
   }
   return true;
}

// An input port network, by its buffer: whether it has the shape above,
// and its nodes
struct port_network_t {
   bool ok;
   set<ivl_lpm_t> nodes;
};
static map<ivl_net_logic_t, port_network_t> g_port_networks;

/*
 * Check the part of a network below nexus `nex', which `drv' (the buffer or
 * a node) drives: it must end on input ports of the group of `grp', through
 * core temporaries of the group and network nodes only. `ports' counts the
 * port nets reached.
 */
static bool network_below(ivl_nexus_t nex, const void *drv, ivl_scope_t grp,
                          int depth, port_network_t &net, unsigned &ports)
{
   if (nex == NULL || depth > 8)
      return false;
   const ivl_scope_t parent = ivl_scope_parent(grp);
   if (group_input_port(nex, grp) != NULL) {
      // A port's net: anything of the module(s) on it, and core temporaries
      // of the parent (the core's resize result) -- nothing else of the
      // parent
      for (unsigned i = 0; i < ivl_nexus_ptrs(nex); i++) {
         ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
         ivl_signal_t s = ivl_nexus_ptr_sig(p);
         ivl_net_logic_t l = ivl_nexus_ptr_log(p);
         ivl_lpm_t m = ivl_nexus_ptr_lpm(p);
         ivl_net_const_t c = ivl_nexus_ptr_con(p);
         ivl_switch_t w = ivl_nexus_ptr_switch(p);
         if ((l != NULL && (const void*)l == drv)
             || (m != NULL && (const void*)m == drv))
            continue;
         if (s != NULL && ivl_signal_local(s) && ivl_signal_scope(s) == parent)
            continue;
         ivl_scope_t sc = s ? ivl_signal_scope(s) : l ? ivl_logic_scope(l)
            : m ? ivl_lpm_scope(m) : c ? ivl_const_scope(c)
            : w ? ivl_switch_scope(w) : NULL;
         if (sc == NULL || !within_instance_group(sc, grp))
            return false;
      }
      ports++;
      return true;
   }

   // A temporary between the buffer and the ports (sv2vhdl mode; plain
   // -tvhdl only draws a buffer straight onto the port): core temporaries,
   // its driver and the network nodes it feeds -- nothing else
   if (!get_sv2vhdl_mode())
      return false;
   bool feeds = false;
   for (unsigned i = 0; i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_signal_t s = ivl_nexus_ptr_sig(p);
      ivl_net_logic_t l = ivl_nexus_ptr_log(p);
      ivl_lpm_t m = ivl_nexus_ptr_lpm(p);
      if (s != NULL) {
         if (!ivl_signal_local(s) || !in_instance_group(ivl_signal_scope(s), grp))
            return false;
      }
      else if (l != NULL) {
         if ((const void*)l != drv)
            return false;
      }
      else if (m != NULL) {
         if ((const void*)m == drv)
            continue;
         ivl_nexus_t in;
         if (!in_instance_group(ivl_lpm_scope(m), grp)
             || !network_node_kind(m, in) || in != nex || ivl_lpm_q(m) == nex)
            return false;
         net.nodes.insert(m);
         if (!network_below(ivl_lpm_q(m), m, grp, depth + 1, net, ports))
            return false;
         feeds = true;
      }
      else
         return false;                   // a constant or a switch on it
   }
   return feeds;
}

// The network a transparent buffer roots (check the shape once)
static const port_network_t &port_network(ivl_net_logic_t log)
{
   map<ivl_net_logic_t, port_network_t>::iterator it = g_port_networks.find(log);
   if (it != g_port_networks.end())
      return it->second;
   port_network_t &net = g_port_networks[log];
   net.ok = false;
   const ivl_scope_t grp = ivl_logic_scope(log);
   if (ivl_logic_type(log) != IVL_LO_BUFT || grp == NULL
       || ivl_scope_type(grp) != IVL_SCT_MODULE || ivl_scope_parent(grp) == NULL
       || !from_outside(ivl_logic_pin(log, 1), grp))
      return net;
   unsigned ports = 0;
   if (network_below(ivl_logic_pin(log, 0), log, grp, 0, net, ports) && ports > 0)
      net.ok = true;
   else
      net.nodes.clear();
   return net;
}

// True if `log' is the buffer of an input port network: drawn in the parent,
// by map_signal, once per instance.
bool is_input_port_buffer(ivl_net_logic_t log)
{
   return ivl_logic_type(log) == IVL_LO_BUFT && port_network(log).ok;
}

// The buffer of the network that `lpm' would be a node of, or NULL
static ivl_net_logic_t network_root_of(ivl_lpm_t lpm, int depth)
{
   ivl_nexus_t in;
   if (depth > 8 || !network_node_kind(lpm, in)
       || group_input_port(in, ivl_lpm_scope(lpm)) != NULL)
      return NULL;                       // (a select of a port in the module)
   for (unsigned i = 0; i < ivl_nexus_ptrs(in); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(in, i);
      ivl_net_logic_t l = ivl_nexus_ptr_log(p);
      if (l != NULL && ivl_logic_type(l) == IVL_LO_BUFT && ivl_logic_pin(l, 0) == in)
         return l;
      ivl_lpm_t m = ivl_nexus_ptr_lpm(p);
      if (m != NULL && m != lpm && ivl_lpm_q(m) == in)
         return network_root_of(m, depth + 1);
   }
   return NULL;
}

// True if `lpm' is a node of an input port network (a pad, prune or split of
// an actual): drawn in the parent with its buffer
bool is_input_port_network_lpm(ivl_lpm_t lpm)
{
   ivl_net_logic_t root = network_root_of(lpm, 0);
   if (root == NULL)
      return false;
   const port_network_t &net = port_network(root);
   return net.ok && net.nodes.count(lpm) != 0;
}

// The buffer of the network that drives the net `nex' of an input port of
// the group of `sc', or NULL
static ivl_net_logic_t port_network_root(ivl_nexus_t nex, ivl_scope_t sc)
{
   for (unsigned i = 0; nex != NULL && i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_net_logic_t l = ivl_nexus_ptr_log(p);
      if (l != NULL && ivl_logic_pin(l, 0) == nex
          && in_instance_group(ivl_logic_scope(l), sc) && is_input_port_buffer(l))
         return l;
      ivl_lpm_t m = ivl_nexus_ptr_lpm(p);
      if (m != NULL && ivl_lpm_q(m) == nex
          && in_instance_group(ivl_lpm_scope(m), sc) && is_input_port_network_lpm(m))
         return network_root_of(m, 0);
   }
   return NULL;
}

/*
 * Is the net `nex' of an input port of the group of `grp' fed from outside
 * the group through core nodes (a buffer, pad, prune, split or cast) and core
 * temporaries? A port so connected is not an open one: if nothing else
 * translates the connection, it is lost.
 */
static bool connected_through_core(ivl_nexus_t nex, ivl_scope_t grp, int depth)
{
   if (nex == NULL || depth > 8)
      return false;
   for (unsigned i = 0; i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t p = ivl_nexus_ptr(nex, i);
      ivl_net_logic_t l = ivl_nexus_ptr_log(p);
      ivl_lpm_t m = ivl_nexus_ptr_lpm(p);
      ivl_nexus_t in = NULL;
      if (l != NULL && ivl_logic_type(l) == IVL_LO_BUFT && ivl_logic_pin(l, 0) == nex)
         in = ivl_logic_pin(l, 1);
      else if (m != NULL && ivl_lpm_q(m) == nex) {
         switch (ivl_lpm_type(m)) {
         case IVL_LPM_PART_VP:
         case IVL_LPM_CONCAT:
         case IVL_LPM_SIGN_EXT:
         case IVL_LPM_CAST_INT:
         case IVL_LPM_CAST_INT2:
         case IVL_LPM_CAST_REAL:
            in = ivl_lpm_data(m, 0);
            break;
         default:
            break;
         }
      }
      if (in == NULL || in == nex)
         continue;
      // Its input: a core temporary of the group (follow it) or a net of the
      // parent (connected); a net of the module is the module's own logic
      bool temp = true, outside = false;
      for (unsigned j = 0; j < ivl_nexus_ptrs(in); j++) {
         ivl_nexus_ptr_t q = ivl_nexus_ptr(in, j);
         ivl_signal_t s = ivl_nexus_ptr_sig(q);
         ivl_net_const_t c = ivl_nexus_ptr_con(q);
         if (s != NULL) {
            if (!within_instance_group(ivl_signal_scope(s), grp))
               outside = true;
            else if (!ivl_signal_local(s) && ivl_signal_port(s) != IVL_SIP_INPUT)
               temp = false;
         }
         else if (c != NULL && !within_instance_group(ivl_const_scope(c), grp))
            outside = true;
      }
      if (!temp)
         continue;
      if (outside || connected_through_core(in, grp, depth + 1))
         return true;
   }
   return false;
}

/*
 * A transparent buffer of the core's port-buffer shape -- in a module
 * instance, at the instantiation's own source line, its input from outside
 * the instance, its output on a core temporary or an input port -- that is
 * not the buffer of a network of the shape above (for instance one that
 * feeds a cast of the parent): its connection would be lost. Say so (an
 * error) and return true.
 */
bool untranslated_port_buffer(ivl_net_logic_t log)
{
   if (ivl_logic_type(log) != IVL_LO_BUFT || is_input_port_buffer(log))
      return false;
   const ivl_scope_t grp = ivl_logic_scope(log);
   if (grp == NULL || ivl_scope_type(grp) != IVL_SCT_MODULE
       || ivl_scope_parent(grp) == NULL
       || ivl_logic_lineno(log) != ivl_scope_lineno(grp)
       || strcmp(ivl_logic_file(log), ivl_scope_file(grp)) != 0
       || !from_outside(ivl_logic_pin(log, 1), grp))
      return false;
   ivl_nexus_t out = ivl_logic_pin(log, 0);
   for (unsigned i = 0; out != NULL && i < ivl_nexus_ptrs(out); i++) {
      ivl_signal_t s = ivl_nexus_ptr_sig(ivl_nexus_ptr(out, i));
      if (s != NULL && !ivl_signal_local(s) && ivl_signal_port(s) != IVL_SIP_INPUT)
         return false;
   }
   error("%s:%d: an input port connection of instance %s is not translated: "
         "the iverilog core buffers its actual in a shape tgt-vhdl does not "
         "draw (a type cast of the connection?)", ivl_logic_file(log),
         ivl_logic_lineno(log), ivl_scope_name(grp));
   return true;
}

/*
 * Is the net of input port `sig' driven inside its module: by a gate, LPM or
 * constant of the module or of an instance below it, by a variable of such
 * an instance (an output reg port), or by a port of an instance below that
 * its entity declares inout (an inout port the module drives, or an input
 * port driven inside in turn)? Its own port buffer and network do not count:
 * that is the actual's drive. A switch does not count either (tran_vp_alias
 * keeps the input one way, with a warning). An `in' VHDL port cannot carry
 * such a driver -- nvc rejects the association of an `in' port with an
 * `inout' or `out' formal, and an assignment to it -- so the port becomes
 * inout. This happens where the iverilog core keeps the port an input: an
 * elaboration root (sv2vhdl-modules translates every module as one), an
 * instance whose actual is a variable or an expression (the port networks
 * above), and a port whose module drives it through a child's inout port
 * only (the core does not see such a driver).
 */
static bool input_driven_inside_(ivl_signal_t sig)
{
   const ivl_scope_t sc = ivl_signal_scope(sig);
   const ivl_nexus_t nex = ivl_signal_nex(sig, 0);
   if (nex == NULL)
      return false;
   for (unsigned i = 0; i < ivl_nexus_ptrs(nex); i++) {
      ivl_nexus_ptr_t ptr = ivl_nexus_ptr(nex, i);
      ivl_net_logic_t log = ivl_nexus_ptr_log(ptr);
      if (log != NULL && ivl_logic_pin(log, 0) == nex
          && scope_within(ivl_logic_scope(log), sc) && !is_input_port_buffer(log))
         return true;
      ivl_lpm_t lpm = ivl_nexus_ptr_lpm(ptr);
      if (lpm != NULL && ivl_lpm_q(lpm) == nex
          && scope_within(ivl_lpm_scope(lpm), sc)
          && !is_input_port_network_lpm(lpm))
         return true;
      ivl_net_const_t con = ivl_nexus_ptr_con(ptr);
      if (con != NULL && scope_within(ivl_const_scope(con), sc))
         return true;
      ivl_signal_t s = ivl_nexus_ptr_sig(ptr);
      if (s == NULL || s == sig || ivl_signal_scope(s) == sc
          || !scope_within(ivl_signal_scope(s), sc))
         continue;
      if (ivl_signal_type(s) == IVL_SIT_REG)
         return true;
      if (ivl_signal_port(s) == IVL_SIP_INOUT
          && (!get_sv2vhdl_mode() || inout_driven_internally(s)))
         return true;
      if (ivl_signal_port(s) == IVL_SIP_INPUT && input_driven_inside(s))
         return true;
   }
   return false;
}

// input_driven_inside_, once per port (a chain of input ports would make it
// exponential); a cycle reads `not driven'
static map<ivl_signal_t, int> g_driven_input_memo;

static bool input_driven_inside(ivl_signal_t sig)
{
   if (ivl_signal_port(sig) != IVL_SIP_INPUT)
      return false;
   map<ivl_signal_t, int>::const_iterator m = g_driven_input_memo.find(sig);
   if (m != g_driven_input_memo.end())
      return m->second == 1;
   g_driven_input_memo[sig] = 0;
   const bool driven = input_driven_inside_(sig);
   g_driven_input_memo[sig] = driven ? 1 : 0;
   return driven;
}

// Input ports declared inout because input_driven_inside
static set<ivl_signal_t> g_driven_inputs;

/*
 * The VHDL name of each entity port, by the Verilog port name. It is not
 * always make_safe_name of the Verilog name: a port whose name differs only
 * in case from an earlier one gets a suffix (`out' and `OUT' become out_sig
 * and OUT_sig_1), and the signal behind an output reg port is renamed to
 * <port>_Reg. map_signal names the formal of an instance port from here.
 */
static map<const vhdl_entity*, map<string, string> > g_port_names;

static void remember_port_name(const vhdl_entity *ent, ivl_signal_t sig,
                               const string &vhdl_name)
{
   g_port_names[ent][ivl_signal_basename(sig)] = vhdl_name;
}

// The formal for port `to' of a child instance: the name its entity declares
static string port_formal_name(ivl_signal_t to)
{
   const vhdl_entity *child = find_entity(ivl_signal_scope(to));
   map<const vhdl_entity*, map<string, string> >::const_iterator e =
      g_port_names.find(child);
   if (e != g_port_names.end()) {
      map<string, string>::const_iterator p =
         e->second.find(ivl_signal_basename(to));
      if (p != e->second.end())
         return p->second;
   }
   return make_safe_name(to);
}

static void declare_one_signal(vhdl_entity *ent, ivl_signal_t sig,
   ivl_scope_t scope)
{
   remember_signal(sig, ent->get_arch()->get_scope());

   string name(make_safe_name(sig));
   name += genvar_unique_suffix(scope);
   avoid_name_collision(name, ent->get_arch()->get_scope());

   rename_signal(sig, name);
   if (ivl_signal_port(sig) != IVL_SIP_NONE)
      remember_port_name(ent, sig, name);

   // SystemVerilog dynamic queue ([$]): model as a bounded ring buffer —
   // a fixed array plus head/tail integer cursors. push_back advances tail,
   // delete(0)/pop_front advances head; because they touch *different*
   // signals a same-cycle push+pop composes correctly under NBA semantics.
   // size = tail-head; element i = arr((head+i) mod DEPTH). See expr.cc
   // ($size, indexing) and stmt.cc ($ivl_queue_method$*, "= {}" clear).
   if (ivl_signal_data_type(sig) == IVL_VT_QUEUE) {
      const int qdepth = 64;   // bound: max in-flight items (>= FIFO capacity)
      // ivl_signal_width() is 1 for a queue; the element width comes from the
      // queue's element type (e.g. logic [3:0] q[$] -> element width 4).
      int elem_w = 1;
      ivl_type_t qtype = ivl_signal_net_type(sig);
      ivl_type_t etype = qtype ? ivl_type_element(qtype) : 0;
      if (etype)
         elem_w = ivl_type_packed_width(etype);
      if (elem_w < 1) elem_w = 1;
      vhdl_type *base =
         vhdl_type::type_for(elem_w, ivl_signal_signed(sig) != 0);
      string type_name = name + "_QType";
      const vhdl_type *arr_type =
         vhdl_type::array_of(base, type_name, qdepth - 1, 0);
      ent->get_arch()->get_scope()->add_decl(
         new vhdl_type_decl(type_name, arr_type));
      ent->get_arch()->get_scope()->add_decl(
         new vhdl_signal_decl(name, new vhdl_type(*arr_type)));

      vhdl_signal_decl *qhead =
         new vhdl_signal_decl(name + "_head", new vhdl_type(VHDL_TYPE_INTEGER));
      qhead->set_initial(new vhdl_const_int(0));
      ent->get_arch()->get_scope()->add_decl(qhead);
      vhdl_signal_decl *qtail =
         new vhdl_signal_decl(name + "_tail", new vhdl_type(VHDL_TYPE_INTEGER));
      qtail->set_initial(new vhdl_const_int(0));
      ent->get_arch()->get_scope()->add_decl(qtail);
      return;
   }

   const vhdl_type *sig_type;
   unsigned dimensions = ivl_signal_dimensions(sig);
   if (dimensions > 0) {
      // Arrays are implemented by generating a separate type
      // declaration for each array, and then declaring a
      // signal of that type

      if (dimensions > 1) {
         error("> 1 dimension arrays not implemented yet");
         return;
      }

      string type_name = name + "_Type";
      vhdl_type *base_type;
      if (ivl_signal_data_type(sig) == IVL_VT_REAL)
         base_type = vhdl_type::real();   // real array: element is VHDL real
      else
         base_type = vhdl_type::type_for(ivl_signal_width(sig),
                                         ivl_signal_signed(sig) != 0);

      // Every word address the core hands us is CANONICAL: 0 for the
      // word with the lowest Verilog index, whatever the declared range
      // (ivl_expr_oper1 of a word read, ivl_lval_idx, ivl_signal_nex pins;
      // normalize_variable_unpacked subtracts min(msb, lsb)). So the VHDL
      // array is (count-1 downto 0): keeping the Verilog range ([4:7],
      // [-2:1]) indexed every access 0-based into it, so a write vanished
      // or landed on the wrong word and a read gave x or stopped the run.
      int lsb = 0;
      int msb = ivl_signal_array_count(sig) - 1;

      const vhdl_type *array_type =
         vhdl_type::array_of(base_type, type_name, msb, lsb);
      vhdl_decl *array_decl = new vhdl_type_decl(type_name, array_type);
      ent->get_arch()->get_scope()->add_decl(array_decl);

      sig_type = new vhdl_type(*array_type);
   }
   else if (ivl_signal_data_type(sig) == IVL_VT_REAL) {
      // Verilog real/realtime -> VHDL real. Width-based type_for would see
      // width 1 and pick logic3d.
      sig_type = vhdl_type::real();
   }
   else {
      sig_type = vhdl_type::type_for(ivl_signal_width(sig),
                                     ivl_signal_signed(sig) != 0,
                                     0, ivl_signal_type(sig) == IVL_SIT_UWIRE);


   }

   ivl_signal_port_t mode = ivl_signal_port(sig);
   switch (mode) {
   case IVL_SIP_NONE:
      {
         vhdl_decl *decl = new vhdl_signal_decl(name, sig_type);

         // A multiply-driven / bidirectional net uses the resolved logic3d
         // subtype so its drivers (and any external one via an inout port)
         // combine through l3d_resolve.
         if (net_needs_resolution(sig))
            decl->set_resolved(true);

         ostringstream ss;
         if (ivl_signal_local(sig)) {
               ss << "Temporary created at " << ivl_signal_file(sig) << ":"
                  << ivl_signal_lineno(sig);
         } else {
            ss << "Declared at " << ivl_signal_file(sig) << ":"
               << ivl_signal_lineno(sig);
         }
         decl->set_comment(ss.str());

         ent->get_arch()->get_scope()->add_decl(decl);
      }
         break;
   case IVL_SIP_INPUT:
      if (input_driven_inside(sig)) {
         // Driven inside the module too (see input_driven_inside): inout,
         // resolved like a driven inout port. The actual's drive still
         // reaches the net, from the parent (map_signal).
         vhdl_port_decl *pd =
            new vhdl_port_decl(name.c_str(), sig_type, VHDL_PORT_INOUT);
         if (get_sv2vhdl_mode())
            pd->set_resolved(true);
         ent->get_scope()->add_decl(pd);
         g_driven_inputs.insert(sig);
      }
      else
         ent->get_scope()->add_decl
            (new vhdl_port_decl(name.c_str(), sig_type, VHDL_PORT_IN));
      break;
   case IVL_SIP_OUTPUT:
      {
         vhdl_port_decl *decl =
            new vhdl_port_decl(name.c_str(), sig_type, VHDL_PORT_OUT);
         ent->get_scope()->add_decl(decl);
      }

      if (ivl_signal_type(sig) == IVL_SIT_REG) {
         // A registered output
         // In Verilog the output and reg can have the
         // same name: this is not valid in VHDL
         // Instead a new signal foo_Reg is created
         // which represents the register
         std::string newname(name);
         newname += "_Reg";
         rename_signal(sig, newname);

         const vhdl_type *reg_type = new vhdl_type(*sig_type);
         ent->get_arch()->get_scope()->add_decl
            (new vhdl_signal_decl(newname, reg_type));

         // Create a concurrent assignment statement to
         // connect the register to the output
         ent->get_arch()->add_stmt
            (new vhdl_cassign_stmt
             (new vhdl_var_ref(name, NULL),
              new vhdl_var_ref(newname, NULL)));
         }
      break;
   case IVL_SIP_INOUT:
      {
         // nvc reads an inout port's own (default) driver, not the external
         // value, so a read-only inout Verilog port must be `in` to see the
         // connected value. Keep `inout` (resolved) only when the module
         // actually drives the port.
         bool driven = inout_driven_internally(sig);
         vhdl_port_mode_t mode = (get_sv2vhdl_mode() && !driven)
            ? VHDL_PORT_IN : VHDL_PORT_INOUT;
         vhdl_port_decl *pd = new vhdl_port_decl(name.c_str(), sig_type, mode);
         if (get_sv2vhdl_mode() && mode == VHDL_PORT_INOUT)
            pd->set_resolved(true);
         ent->get_scope()->add_decl(pd);
      }
      break;
   default:
      assert(false);
   }

   // In sv2vhdl mode, emit discipline/nature attributes for analog signals
   ivl_discipline_t disc = ivl_signal_discipline(sig);
   if (disc && get_sv2vhdl_mode()) {
      ent->get_arch()->add_attribute_spec("discipline", name, "signal",
                                           ivl_discipline_name(disc));
      ivl_nature_t pot = ivl_discipline_potential(disc);
      if (pot)
         ent->get_arch()->add_attribute_spec("va_nature_potential", name,
                                              "signal", ivl_nature_name(pot));
      ivl_nature_t flow = ivl_discipline_flow(disc);
      if (flow)
         ent->get_arch()->add_attribute_spec("va_nature_flow", name,
                                              "signal", ivl_nature_name(flow));
   }
}

// Declare all signals and ports for a scope.
// This is done in two phases: first the ports are added, then
// internal signals. Making two passes like this ensures ports get
// first pick of names when there is a collision.
static void declare_signals(vhdl_entity *ent, ivl_scope_t scope)
{
   debug_msg("Declaring signals in scope type %s", ivl_scope_tname(scope));

   int nsigs = ivl_scope_sigs(scope);
   // Emit ports in module-declaration order so that positional port
   // maps bind correctly.  Fall back to signal-table order for
   // non-module scopes or unusual port declarations.
   // Try to emit ports in module-declaration order for correct positional
   // port map binding.  Fall back to signal-table order if anything goes
   // wrong (unusual port declarations, concatenated ports, etc.)
   bool used_port_order = false;
   if (ivl_scope_type(scope) == IVL_SCT_MODULE) {
      const unsigned nports = ivl_scope_mod_module_ports(scope);
      if (nports > 0) {
         unsigned matched = 0;
         std::set<std::string> seen_ports;
         for (unsigned p = 0; p < nports; p++) {
            const char *pname = ivl_scope_mod_module_port_name(scope, p);
            if (!pname || !seen_ports.insert(pname).second)
               continue;
            for (int i = 0; i < nsigs; i++) {
               ivl_signal_t sig = ivl_scope_sig(scope, i);
               if (ivl_signal_port(sig) != IVL_SIP_NONE
                   && strcmp(ivl_signal_basename(sig), pname) == 0) {
                  declare_one_signal(ent, sig, scope);
                  matched++;
                  break;
               }
            }
         }
         used_port_order = (matched > 0);
      }
   }
   if (!used_port_order) {
      for (int i = 0; i < nsigs; i++) {
         ivl_signal_t sig = ivl_scope_sig(scope, i);
         if (ivl_signal_port(sig) != IVL_SIP_NONE)
            declare_one_signal(ent, sig, scope);
      }
   }

   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t sig = ivl_scope_sig(scope, i);

      if (ivl_signal_port(sig) == IVL_SIP_NONE)
         declare_one_signal(ent, sig, scope);
   }
}

/*
 * Generate VHDL for LPM instances in a module.
 */
static void declare_lpm(vhdl_arch *arch, ivl_scope_t scope)
{
   int nlpms = ivl_scope_lpms(scope);
   for (int i = 0; i < nlpms; i++) {
      ivl_lpm_t lpm = ivl_scope_lpm(scope, i);
      // A node of an input port network: drawn in the parent (map_signal)
      if (is_input_port_network_lpm(lpm))
         continue;
      if (draw_lpm(arch, lpm) != 0)
         error("Failed to translate LPM %s", ivl_lpm_name(lpm));
   }
}

// A new signal of the parent standing for a net of an input port
// connection: <prefix>_<label>_<port>[_<n>]
static vhdl_signal_decl *port_buffer_decl(vhdl_scope *ascope,
                                          const string &prefix,
                                          vhdl_comp_inst *inst,
                                          const string &formal,
                                          const vhdl_type *type, bool resolved,
                                          const string &comment)
{
   string stem = prefix + "_" + inst->get_inst_name() + "_" + formal;
   replace_consecutive_underscores(stem);
   string tname = stem;
   for (int u = 2; ascope->have_declared(tname); u++) {
      ostringstream ss;
      ss << stem << "_" << u;
      tname = ss.str();
   }
   vhdl_signal_decl *decl = new vhdl_signal_decl(tname, type);
   if (resolved)
      decl->set_resolved(true);
   decl->set_comment(comment);
   ascope->add_decl(decl);
   return decl;
}

// The parent's expression of the actual of network buffer `buf': a signal of
// the parent, or the constant the actual is; NULL if it has none
static vhdl_expr *buffer_actual(vhdl_scope *ascope, ivl_net_logic_t buf)
{
   ivl_nexus_t actual = ivl_logic_pin(buf, 1);
   seen_nexus(actual);
   nexus_private_t *priv =
      static_cast<nexus_private_t*>(ivl_nexus_get_private(actual));
   if (visible_nexus(priv, ascope))
      return nexus_to_var_ref(ascope, actual);
   if (priv->const_driver != NULL) {
      vhdl_expr *c = priv->const_driver;
      priv->const_driver = NULL;
      return c;
   }
   return NULL;
}

// The actual of network buffer `buf' as a plain logic signal of the parent
// (no select of it, no constant), or NULL
static vhdl_var_ref *plain_actual_ref(vhdl_scope *ascope, ivl_net_logic_t buf)
{
   ivl_nexus_t actual = ivl_logic_pin(buf, 1);
   seen_nexus(actual);
   nexus_private_t *priv =
      static_cast<nexus_private_t*>(ivl_nexus_get_private(actual));
   if (!visible_nexus(priv, ascope))
      return NULL;
   vhdl_var_ref *r = nexus_to_var_ref(ascope, actual);
   const vhdl_type *t = r->get_type();
   if (r->get_slice() != NULL || t == NULL
       || (t->get_name() != VHDL_TYPE_LOGIC3D
           && t->get_name() != VHDL_TYPE_LOGIC3D_VECTOR))
      return NULL;
   return r;
}

// The value a node of an input port network gives, from its input `in'
static vhdl_expr *network_node_expr(ivl_lpm_t node, vhdl_var_ref *in)
{
   const int w = ivl_lpm_width(node);
   const vhdl_type *it = in->get_type();
   const bool vec = it != NULL && it->get_name() == VHDL_TYPE_LOGIC3D_VECTOR;
   switch (ivl_lpm_type(node)) {
   case IVL_LPM_PART_VP:                 // a prune, or an element's part
      if (vec)
         in->set_slice(new vhdl_const_int(ivl_lpm_base(node)), w - 1);
      return in;
   case IVL_LPM_SIGN_EXT: {              // a signed pad
      if (!vec)
         return new vhdl_bit_spec_expr(NULL, in);
      vhdl_fcall *f = new vhdl_fcall("l3d_resize_s",
                                     vhdl_type::logic3d_vector(w - 1, 0));
      f->add_expr(in);
      f->add_expr(new vhdl_const_int(w));
      return f;
   }
   default: {                            // a zero pad (IVL_LPM_CONCAT)
      const int npad = w - (vec ? it->get_width() : 1);
      vhdl_expr *zeros = npad == 1 ? (vhdl_expr*)new vhdl_const_bit('0')
         : new vhdl_const_bits(string(npad, '0').c_str(), npad, false, true);
      vhdl_binop_expr *cat = new vhdl_binop_expr(VHDL_BINOP_CONCAT,
         vhdl_type::logic3d_vector(w - 1, 0));
      cat->add_expr(zeros);
      cat->add_expr(in);
      return cat;
   }
   }
}

/*
 * The parent's signal for net `nex' of the input port network of buffer
 * `root', mapping port `port' of instance `inst': drawn on first use, with
 * what drives it -- the buffer (a copy of the actual), or a pad, prune or
 * split of the net before it, drawn the same way. A port's net is a resolved
 * PB_<label>_<port>, which also carries the pull of a tri1/tri0 port net (no
 * plain signal of the net is left for draw_constant_drivers to put it on); a
 * temporary between is a PBT_<label>_<port>. NULL if it cannot be drawn.
 */
// The network nets network_parent_ref has drawn the driver of
static set<ivl_nexus_t> g_network_drawn;

static vhdl_var_ref *network_parent_ref(vhdl_arch *arch, ivl_nexus_t nex,
                                        ivl_net_logic_t root,
                                        ivl_signal_t port,
                                        vhdl_comp_inst *inst, int depth)
{
   vhdl_scope *ascope = arch->get_scope();
   seen_nexus(nex);
   nexus_private_t *priv =
      static_cast<nexus_private_t*>(ivl_nexus_get_private(nex));
   assert(priv);
   // Drawn already (by an earlier element of an instance array); or a core
   // temporary of the parent stands for the net already, without a driver
   const bool visible = visible_nexus(priv, ascope) != NULL;
   if (visible && g_network_drawn.count(nex))
      return nexus_to_var_ref(ascope, nex);
   if (depth > 8)
      return NULL;

   // What drives it: the buffer, or a node of its network
   const port_network_t &net = port_network(root);
   ivl_lpm_t node = NULL;
   if (ivl_logic_pin(root, 0) != nex) {
      for (unsigned i = 0; node == NULL && i < ivl_nexus_ptrs(nex); i++) {
         ivl_lpm_t m = ivl_nexus_ptr_lpm(ivl_nexus_ptr(nex, i));
         if (m != NULL && ivl_lpm_q(m) == nex && net.nodes.count(m))
            node = m;
      }
      if (node == NULL)
         return NULL;
   }
   vhdl_expr *src = NULL;
   if (node == NULL)
      src = buffer_actual(ascope, root);
   else {
      ivl_nexus_t in;
      network_node_kind(node, in);
      // A node right after the buffer reads the actual itself when that is
      // a plain signal of the parent (`PB_s1_scl <= v(1)', `PB_u_a <= L3D_0
      // & r'), not a PBT_ copy of it
      vhdl_var_ref *iref = NULL;
      if (in == ivl_logic_pin(root, 0)
          && group_input_port(in, ivl_logic_scope(root)) == NULL)
         iref = plain_actual_ref(ascope, root);
      if (iref == NULL)
         iref = network_parent_ref(arch, in, root, port, inst, depth + 1);
      if (iref != NULL)
         src = network_node_expr(node, iref);
   }
   if (src == NULL)
      return NULL;
   g_network_drawn.insert(nex);
   if (visible) {
      // (that temporary is a plain signal of the parent: draw_constant_drivers
      // puts a pull of the net on it)
      vhdl_var_ref *target = nexus_to_var_ref(ascope, nex);
      arch->add_stmt(new vhdl_cassign_stmt(target, src->cast(target->get_type())));
      return nexus_to_var_ref(ascope, nex);
   }

   const ivl_signal_t pport = group_input_port(nex, ivl_logic_scope(root));
   ostringstream cs;
   vhdl_signal_decl *decl;
   if (pport != NULL) {
      cs << "Port buffer of input " << ivl_signal_basename(pport)
         << " of instance " << ivl_scope_basename(ivl_signal_scope(port))
         << " (its net is also driven inside)";
      decl = port_buffer_decl(ascope, "PB", inst, port_formal_name(pport),
                              vhdl_type_for_signal(pport),
                              get_sv2vhdl_mode()
                              && ivl_signal_data_type(pport) != IVL_VT_REAL,
                              cs.str());
   }
   else {
      ivl_signal_t ls = NULL;
      for (unsigned i = 0; ls == NULL && i < ivl_nexus_ptrs(nex); i++)
         ls = ivl_nexus_ptr_sig(ivl_nexus_ptr(nex, i));
      const vhdl_type *type = ls != NULL ? vhdl_type_for_signal(ls)
         : vhdl_type::type_for(node ? ivl_lpm_width(node)
                                    : ivl_logic_width(root), false);
      cs << "Connection of input " << ivl_signal_basename(port)
         << " of instance " << ivl_scope_basename(ivl_signal_scope(port))
         << ", before its pad, prune or instance-array split";
      decl = port_buffer_decl(ascope, "PBT", inst, port_formal_name(port),
                              type, false, cs.str());
   }
   link_scope_to_nexus_tmp(priv, ascope, decl->get_name());
   arch->add_stmt(new vhdl_cassign_stmt(decl->make_ref(),
                                        src->cast(decl->get_type())));
   if (pport != NULL && priv->pull != 0) {
      emit_tri_pull(arch, decl->make_ref(), priv->pull);
      priv->pull = 0;
   }
   return decl->make_ref();
}

// The child entity declares the formal of port `to' inout
static bool child_formal_inout(ivl_signal_t to, const string &formal)
{
   vhdl_entity *child = find_entity(ivl_signal_scope(to));
   if (child == NULL)
      return false;
   const vhdl_port_decl *pd =
      dynamic_cast<const vhdl_port_decl*>(child->get_scope()->get_decl(formal));
   return pd != NULL && pd->get_mode() == VHDL_PORT_INOUT;
}

// Port `to' of instance `inst' (declared inout, its net driven inside) on a
// resolved PB_<label>_<port> of the parent that a one-way copy of `src'
// drives, or (src NULL) that pull `pull' pulls
static void map_port_copy(vhdl_arch *arch, ivl_signal_t to,
                          vhdl_comp_inst *inst, const string &formal,
                          vhdl_expr *src, char pull)
{
   const vhdl_type *type = vhdl_type_for_signal(to);
   ostringstream cs;
   cs << "Port buffer of input " << ivl_signal_basename(to) << " of instance "
      << ivl_scope_basename(ivl_signal_scope(to))
      << " (its net is also driven inside)";
   vhdl_signal_decl *decl =
      port_buffer_decl(arch->get_scope(), "PB", inst, formal, type,
                       get_sv2vhdl_mode()
                       && ivl_signal_data_type(to) != IVL_VT_REAL, cs.str());
   if (src != NULL)
      arch->add_stmt(new vhdl_cassign_stmt(decl->make_ref(), src->cast(type)));
   else if (pull != 0)
      emit_tri_pull(arch, decl->make_ref(), pull);
   inst->map_port(formal, decl->make_ref());
}

/*
 * Port `to' of instance `inst' on a concatenation (concat_port_parts): its
 * formal associated part by part, each with the operand the part joins, so
 * the port and every operand are joined both ways:
 *
 *    y(3) => p, y(2) => q, y(1 downto 0) => bus_sig(5 downto 4)
 *
 * (an operand that is a select is associated with that part of the vector).
 * An `in' or `out' port of the parent cannot be the actual of an inout
 * formal: a resolved PB_ signal and a one-way copy stand in for it, with a
 * warning.
 */
static void map_concat_parts(vhdl_arch *arch, ivl_signal_t to,
                             vhdl_comp_inst *inst, const string &formal,
                             const vector<ivl_switch_t> &parts)
{
   vhdl_scope *ascope = arch->get_scope();
   const bool inout = child_formal_inout(to, formal);
   for (size_t i = 0; i < parts.size(); i++) {
      ivl_switch_t sw = parts[i];
      const unsigned off = ivl_switch_offset(sw);
      const unsigned part = ivl_switch_part(sw);
      vhdl_var_ref *act;
      ivl_switch_t sel = concat_operand_select(sw);
      if (sel != NULL) {
         act = nexus_to_var_ref(ascope, ivl_switch_a(sel));
         if (part == 1)
            act->set_slice(new vhdl_const_int(ivl_switch_offset(sel)));
         else
            act->set_slice(new vhdl_const_int(ivl_switch_offset(sel)), part - 1);
      }
      else
         act = nexus_to_var_ref(ascope, ivl_switch_b(sw));

      ostringstream f;
      f << formal << "(";
      if (part > 1)
         f << off + part - 1 << " downto ";
      f << off << ")";

      vhdl_expr *actual = act;
      const vhdl_port_decl *pd =
         dynamic_cast<const vhdl_port_decl*>(ascope->get_decl(act->get_name()));
      if (inout && pd != NULL && pd->get_mode() != VHDL_PORT_INOUT) {
         const vhdl_type *type = part > 1
            ? vhdl_type::logic3d_vector(part - 1, 0) : vhdl_type::logic3d();
         ostringstream cs, lb;
         cs << "Part " << f.str() << " of inout port " << ivl_signal_basename(to)
            << " of instance " << ivl_scope_basename(ivl_signal_scope(to))
            << " (on the " << (pd->get_mode() == VHDL_PORT_IN ? "input" : "output")
            << " port " << act->get_name() << ")";
         lb << formal << "_" << off;
         vhdl_signal_decl *decl = port_buffer_decl(ascope, "PB", inst, lb.str(),
                                                   type, get_sv2vhdl_mode(),
                                                   cs.str());
         if (pd->get_mode() == VHDL_PORT_IN)
            arch->add_stmt(new vhdl_cassign_stmt(decl->make_ref(),
                                                 act->cast(type)));
         else
            arch->add_stmt(new vhdl_cassign_stmt(act, decl->make_ref()));
         cerr << "Warning: inout port " << ivl_signal_name(to) << " at "
              << ivl_switch_file(sw) << ":" << ivl_switch_lineno(sw)
              << " is connected one way only to " << act->get_name()
              << ": an " << (pd->get_mode() == VHDL_PORT_IN ? "input" : "output")
              << " port of the enclosing module" << endl;
         actual = decl->make_ref();
      }
      inst->map_port(f.str(), actual);
   }
}

// Can `ref', a signal of the parent, be the actual of an inout port: a
// resolved signal, an inout port, an alias (T2: of a resolved vector)?
static bool joinable_actual(vhdl_scope *ascope, const vhdl_var_ref *ref)
{
   const vhdl_decl *d = ascope->get_decl(ref->get_name());
   if (d == NULL || ref->get_slice() != NULL)
      return false;
   if (dynamic_cast<const vhdl_alias_decl*>(d) != NULL)
      return true;
   if (const vhdl_port_decl *pd = dynamic_cast<const vhdl_port_decl*>(d))
      return pd->get_mode() == VHDL_PORT_INOUT;
   return d->is_resolved();
}

/*
 * Map two signals together in an instantiation.
 * The signals are joined by a nexus.
 */
static void map_signal(ivl_signal_t to, const vhdl_entity *parent,
                       vhdl_comp_inst *inst)
{
   // TODO: Work for multiple words
   ivl_nexus_t nexus = ivl_signal_nex(to, 0);

   // ICG2EN site rewiring: a gated clock port of a signature-split
   // child takes the gate's direct clock input as its actual (the
   // child entity's guard supplies the enable via an upward external
   // name).  Only the ICG-ADJACENT site can see that net; pass-through
   // levels (a wrapper whose own clk port carries the same gated net)
   // keep their normal wiring — the repoint at the top of the chain
   // feeds the root down the port association chain.
   {
      ivl_nexus_t root = NULL;
      if (icg2en_site_root(to, &root) && root != NULL
          && nexus_visible_in_scope(parent->get_arch()->get_scope(), root))
         nexus = root;
   }
   seen_nexus(nexus);

   vhdl_scope *arch_scope = parent->get_arch()->get_scope();

   nexus_private_t *priv =
      static_cast<nexus_private_t*>(ivl_nexus_get_private(nexus));
   assert(priv);

   vhdl_expr *map_to = NULL;
   // The formal as the child entity declares it (with its case-collision
   // suffix, if any); it also names the _Readable shadow below
   const string name(port_formal_name(to));

   const ivl_scope_t to_scope = ivl_signal_scope(to);

   // An input port fed by a port network (the core's buffer, and the pad,
   // prune or instance-array split after it): the network is drawn here
   if (ivl_signal_port(to) == IVL_SIP_INPUT && nexus == ivl_signal_nex(to, 0)) {
      ivl_net_logic_t root = port_network_root(nexus, to_scope);
      if (root != NULL) {
         vhdl_var_ref *pb = network_parent_ref(parent->get_arch(), nexus, root,
                                               to, inst, 0);
         if (pb != NULL)
            inst->map_port(name, pb);
         else
            error("%s:%d: input port %s of instance %s: its actual has no "
                  "translation", ivl_scope_file(to_scope),
                  ivl_scope_lineno(to_scope), ivl_signal_basename(to),
                  ivl_scope_name(to_scope));
         return;
      }
   }

   // An inout port on a concatenation: associated part by part (the only
   // object of the parent on its net is draw_nexus's switch temporary)
   if (get_sv2vhdl_mode() && ivl_signal_port(to) == IVL_SIP_INOUT
       && nexus == ivl_signal_nex(to, 0)) {
      vector<ivl_switch_t> parts;
      ivl_scope_t pinst;
      if (concat_port_parts(nexus, parts, pinst) && pinst == to_scope) {
         map_concat_parts(parent->get_arch(), to, inst, name, parts);
         return;
      }
   }

   // We can only map ports to signals or constants
   if (visible_nexus(priv, arch_scope)) {
      vhdl_var_ref *ref = nexus_to_var_ref(parent->get_arch()->get_scope(), nexus);

      // An input port the child declares inout (its net is driven inside,
      // input_driven_inside) on an object of the parent that is no net -- a
      // variable, a gate or LPM output, a temporary: Verilog (and VCS) never
      // coerce a variable to inout, and the port's drive would override its
      // value. A one-way copy, as the port networks draw.
      const scope_nexus_t *vsn = visible_nexus(priv, arch_scope);
      if (ivl_signal_port(to) == IVL_SIP_INPUT && child_formal_inout(to, name)
          && ((vsn->sig != NULL && ivl_signal_type(vsn->sig) == IVL_SIT_REG)
              || !joinable_actual(arch_scope, ref))) {
         map_port_copy(parent->get_arch(), to, inst, name, ref, 0);
         return;
      }

      // If we're mapping a port that targets one of this entity's
      // non-readable (OUT) signals, VHDL won't let us read it directly.
      // The fix is a shadow signal connected to the port via port_map and
      // to the entity OUT via a concurrent assign.  Direction depends on
      // the child port direction:
      //   child OUT -> parent OUT : child drives shadow; shadow drives OUT
      //   child IN  -> parent OUT : OUT (driven elsewhere by some assign)
      //                             must drive shadow so child reads it
      const vhdl_decl* from_decl =
         parent->get_arch()->get_scope()->get_decl(ref->get_name());
      if (!from_decl->is_readable()
          && !arch_scope->have_declared(name + "_Readable")) {
         vhdl_decl* tmp_decl =
            new vhdl_signal_decl(name + "_Readable", ref->get_type());

         tmp_decl->set_comment("Needed to connect outputs");

         arch_scope->add_decl(tmp_decl);

         if (ivl_signal_port(to) == IVL_SIP_INPUT) {
            // Child reads the OUT port -- shadow must follow the OUT.
            // Cannot use a concurrent assign because the OUT itself
            // isn't readable; use an attribute trick via the BUFFER
            // mode is not possible without changing the port mode, so
            // instead emit a process that copies the readable shadow
            // from any driver-source assignment chain via an external
            // alias.  Best we can do here: drive the shadow from the
            // expression that drives the OUT.  Caller-side: the assign
            // statements already write to from_decl; emit a
            // process(all) that copies from_decl into the shadow.
            //
            // The cleanest way is just `tmp_decl <= from_decl` as a
            // concurrent assign -- VHDL does not allow reading an OUT
            // directly, but referencing the port name on the right of
            // a concurrent statement targeting a local signal is valid
            // in many flows.  If the target tool rejects it, fall back
            // to a process that depends on the explicit assign chain.
            parent->get_arch()->add_stmt
               (new vhdl_cassign_stmt(tmp_decl->make_ref(),
                                      from_decl->make_ref()));
         } else {
            // Child drives the OUT -- shadow takes the value and the
            // OUT is driven from the shadow.
            parent->get_arch()->add_stmt
               (new vhdl_cassign_stmt(from_decl->make_ref(),
                                      tmp_decl->make_ref()));
         }

         map_to = tmp_decl->make_ref();
      }
      else
         map_to = ref;
   }
   else if (priv->const_driver && ivl_signal_port(to) == IVL_SIP_INPUT) {
      map_to = priv->const_driver;
      priv->const_driver = NULL;
      // An unconnected tri1/tri0 input: the constant stands for its pull
      // (the child can only read the port)
      const char pull = priv->tri_default ? priv->pull : 0;
      if (priv->tri_default)
         priv->pull = 0;
      // An inout formal takes no constant: a signal holding it (or, for the
      // pull of an unconnected tri1/tri0 input, the pull itself)
      if (child_formal_inout(to, name)) {
         map_port_copy(parent->get_arch(), to, inst, name,
                       pull ? NULL : map_to, pull);
         return;
      }
   }
   else {
      // This nexus isn't attached to anything in the parent. An input port
      // fed from the parent through core nodes is connected all the same:
      // its connection would be lost, which must not pass silently
      if (ivl_signal_port(to) == IVL_SIP_INPUT
          && connected_through_core(nexus, to_scope, 0))
         error("%s:%d: input port %s of instance %s: its connection is not "
               "translated", ivl_scope_file(to_scope),
               ivl_scope_lineno(to_scope), ivl_signal_basename(to),
               ivl_scope_name(to_scope));
      return;
   }

   inst->map_port(name, map_to);
}

/*
 * Find all the port mappings of a module instantiation.
 */
static void port_map(ivl_scope_t scope, const vhdl_entity *parent,
                     vhdl_comp_inst *inst)
{
   // Find all the port mappings
   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t sig = ivl_scope_sig(scope, i);

      ivl_signal_port_t mode = ivl_signal_port(sig);
      switch (mode) {
      case IVL_SIP_NONE:
         // Internal signals don't appear in the port map
         break;
      case IVL_SIP_INPUT:
      case IVL_SIP_OUTPUT:
      case IVL_SIP_INOUT:
         map_signal(sig, parent, inst);
         break;
      default:
         assert(false);
      }
   }

   // ICG2EN: synthetic guard-port associations for signature-covered
   // gated clock ports of this child (see icg2en_map_enables)
   icg2en_map_enables(scope, parent, inst);
}

/*
 * Create a VHDL function from a Verilog function definition.
 */
// The VHDL name of the function defined by Verilog scope `fscope'.
// A function declared inside a generate block is a distinct function per
// generate instance -- sv2v emits one sv2v_cast_<hash> per scope and its
// formal width follows that scope's localparams (VX_csa_tree: WI = W +
// level) -- and generate blocks are flattened into the architecture, so
// it takes the same suffix as the scope's signals. Every declaration and
// call site must go through here.
string vhdl_function_name(ivl_scope_t fscope)
{
   string name(ivl_scope_tname(fscope));
   ivl_scope_t parent = ivl_scope_parent(fscope);
   if (parent != NULL && ivl_scope_type(parent) == IVL_SCT_GENERATE)
      name += genvar_unique_suffix(parent);
   return name;
}

int draw_function_in_entity(ivl_scope_t scope, vhdl_entity *ent)
{
   assert(ivl_scope_type(scope) == IVL_SCT_FUNCTION);

   debug_msg("Generating function %s (%s)", ivl_scope_tname(scope),
             ivl_scope_name(scope));

   const string funcname_str = vhdl_function_name(scope);
   const char *funcname = funcname_str.c_str();

   // Already emitted here (e.g. a package function drawn on demand by an
   // earlier call site in this entity)
   if (ent->get_arch()->get_scope()->have_declared(funcname))
      return 0;

   // The return type is worked out from the output port
   vhdl_function *func = new vhdl_function(funcname, NULL);

   // Set the parent scope of this function to be the containing
   // architecture. This allows us to look up non-local variables
   // referenced in the body, but if we do the `impure' flag must
   // be set on the function
   // (There are actually two VHDL scopes in a function: the local
   // variables and the formal parameters hence the call to get_parent)
   func->get_scope()->get_parent()->set_parent(ent->get_arch()->get_scope());

   // First we add the input/output parameters in order
   int nports = ivl_scope_ports(scope);
   for (int i = 0; i < nports; i++) {
      ivl_signal_t sig = ivl_scope_port(scope, i);

      const vhdl_type *sigtype = vhdl_type_for_signal(sig);

      string signame(make_safe_name(sig));

      switch (ivl_signal_port(sig)) {
      case IVL_SIP_INPUT:
         func->add_param(new vhdl_param_decl(signame.c_str(), sigtype));
         break;
      case IVL_SIP_OUTPUT:
         // The magic variable <funcname>_Result holds the return value
         signame = funcname;
         signame += "_Result";
         func->set_type(new vhdl_type(*sigtype));
         func->get_scope()->add_decl
            (new vhdl_var_decl(signame, sigtype));
         break;
      default:
         // Only expecting inputs and outputs
         assert(false);
      }

      if (!seen_signal_before(sig)) {
         remember_signal(sig, func->get_scope());
         rename_signal(sig, signame);
      }
   }

   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t sig = ivl_scope_sig(scope, i);

      if (ivl_signal_port(sig) == IVL_SIP_NONE) {
         const vhdl_type *sigtype = vhdl_type_for_signal(sig);

         string signame(make_safe_name(sig));
         func->get_scope()->add_decl(
            new vhdl_var_decl(signame, sigtype));

         if (!seen_signal_before(sig)) {
            remember_signal(sig, func->get_scope());
            rename_signal(sig, signame);
         }
      }
   }

   // Non-blocking assignment not allowed in functions
   func->get_scope()->set_allow_signal_assignment(false);

   // The body runs in the function's own scope: %m, the Scope line of
   // $error & co. and $time's units come from it (top.chk, as vvp names it).
   // Restore the caller's scope and entity after: a package function is drawn
   // on demand in the middle of a process (translate_ufunc), whose later
   // statements still need them (a delay after the call dereferenced a NULL
   // active entity and crashed the translation).
   ivl_scope_t prev_scope = get_active_scope();
   vhdl_entity *prev_ent = get_active_entity();
   set_active_scope(scope);
   set_active_entity(ent);
   {
      // a disable of the function (SV `return expr') returns its result
      begin_function_disables(scope, string(funcname) + "_Result");
      draw_stmt(func, func->get_container(), ivl_scope_def(scope));
      end_function_disables();
   }
   set_active_entity(prev_ent);
   set_active_scope(prev_scope);

   // Add a forward declaration too in case it is called by
   // another function that has already been added
   ent->get_arch()->get_scope()->add_forward_decl
      (new vhdl_forward_fdecl(func));

   ostringstream ss;
   ss << "Generated from function " << funcname << " at "
      << ivl_scope_def_file(scope) << ":" << ivl_scope_def_lineno(scope);
   func->set_comment(ss.str());

   ent->get_arch()->get_scope()->add_decl(func);
   return 0;
}


/*
 * Create the signals necessary to expand this task later.
 */
static int draw_function(ivl_scope_t scope, ivl_scope_t parent)
{
   vhdl_entity *ent = find_entity(parent);
   assert(ent);
   return draw_function_in_entity(scope, ent);
}

static int draw_task(ivl_scope_t scope, ivl_scope_t parent)
{
   assert(ivl_scope_type(scope) == IVL_SCT_TASK);

   // Find the containing entity
   vhdl_entity *ent = find_entity(parent);
   assert(ent);

   const char *taskname = ivl_scope_tname(scope);

   int nsigs = ivl_scope_sigs(scope);
   for (int i = 0; i < nsigs; i++) {
      ivl_signal_t sig = ivl_scope_sig(scope, i);
      const vhdl_type *sigtype = vhdl_type_for_signal(sig);

      string signame(make_safe_name(sig));

      // Check this signal isn't declared in the outer scope
      if (ent->get_arch()->get_scope()->have_declared(signame)) {
         signame += "_";
         signame += taskname;
      }

      vhdl_signal_decl *decl = new vhdl_signal_decl(signame, sigtype);

      ostringstream ss;
      ss << "Declared at " << ivl_signal_file(sig) << ":"
         << ivl_signal_lineno(sig) << " (in task " << taskname << ")";
      decl->set_comment(ss.str());

      ent->get_arch()->get_scope()->add_decl(decl);

      remember_signal(sig, ent->get_arch()->get_scope());
      rename_signal(sig, signame);
   }

   return 0;
}

/*
 * A real parameter value as the "--   P = v" comment line gives it: the
 * shortest of %.6g .. %.17g that reads back as exactly `v`. A value that
 * needs no more than the stream's default 6 significant digits prints as
 * before (0.25, 1e-09); one that needs more keeps them (0.2500001), so
 * two different values never print alike (the AMS cut compares them).
 */
static string real_param_text(double v)
{
   char buf[40];
   for (int prec = 6; prec <= 17; prec++) {
      snprintf(buf, sizeof buf, "%.*g", prec, v);
      if (strtod(buf, NULL) == v)
         break;
   }
   return buf;
}

/*
 * Create an empty VHDL entity for a Verilog module.
 */
static void create_skeleton_entity_for(ivl_scope_t scope, int depth)
{
   assert(ivl_scope_type(scope) == IVL_SCT_MODULE);

   // The type name will become the entity name
   const string tname = valid_entity_name(ivl_scope_tname(scope));

   // Verilog does not have the entity/architecture distinction
   // so we always create a pair and associate the architecture
   // with the entity for convenience (this also means that we
   // retain a 1-to-1 mapping of scope to VHDL element)
   vhdl_arch *arch = new vhdl_arch(tname, "from_verilog");
   vhdl_entity *ent = new vhdl_entity(tname, arch, depth);

   // ICG2EN: synthetic guard ports follow from the signature alone so
   // entity and sites can never disagree (see icg2en_add_entity_ports)
   icg2en_add_entity_ports(scope, ent);

   // Record the original Verilog source location as a VHDL attribute so it
   // survives translation (debug, and --accel recovering the Verilog source).
   {
      std::ostringstream vsrc;
      vsrc << ivl_scope_def_file(scope) << ":" << ivl_scope_def_lineno(scope);
      ent->set_verilog_src(vsrc.str());
   }

   // Calculate the VHDL units to use for time values. Delay values are counted
   // in simulation ticks, and a tick is the DESIGN's time precision (the finest
   // over all modules) -- not this scope's -- so a design mixing timescales
   // (e.g. a 10us module beside a 1us one) scales every delay by the same tick.
   ent->set_time_units(ivl_scope_time_units(scope),
                       ivl_design_time_precision(get_vhdl_design()));

   // Build a comment to add to the entity/architecture
   ostringstream ss;
   ss << "Generated from Verilog module " << ivl_scope_tname(scope)
      << " (" << ivl_scope_def_file(scope) << ":"
      << ivl_scope_def_lineno(scope) << ")";

   // Integer parameter actuals, "name=value name=value", recorded as an
   // attribute so --accel can re-synthesize the module with the SAME generics
   // the elaboration used (iverilog monomorphises, so these are the real ones).
   ostringstream params;

   unsigned nparams = ivl_scope_params(scope);
   for (unsigned i = 0; i < nparams; i++) {
      ivl_parameter_t param = ivl_scope_param(scope, i);

      // Type parameter usages get replaced with their actual type
      if (ivl_parameter_is_type(param))
	    continue;

      ivl_expr_t value = ivl_parameter_expr(param);

      ss << "\n  " << ivl_parameter_basename(param) << " = ";

      switch (ivl_expr_type(value)) {
         case IVL_EX_STRING:
            ss << "\"" << ivl_expr_string(value) << "\"";
            break;

         case IVL_EX_NUMBER:
            ss << ivl_expr_uvalue(value);
            if (!params.str().empty()) params << " ";
            params << ivl_parameter_basename(param) << "="
                   << ivl_expr_uvalue(value);
            break;

         case IVL_EX_REALNUM:
            ss << real_param_text(ivl_expr_dvalue(value));
            break;

      default:
         assert(false);
      }
   }

   ent->set_verilog_params(params.str());

   arch->set_comment(ss.str());
   ent->set_comment(ss.str());

   remember_entity(ent, scope);
}

/*
 * A first pass through the hierarchy: create VHDL entities for
 * each unique Verilog module type.
 */
extern "C" int draw_skeleton_scope(ivl_scope_t scope, void *)
{
   static int depth = 0;

   if (seen_this_scope_type(scope))
      return 0;  // Already generated a skeleton for this scope type

   debug_msg("Initial visit to scope type %s at depth %d",
             ivl_scope_tname(scope), depth);

   switch (ivl_scope_type(scope)) {
   case IVL_SCT_MODULE:
      create_skeleton_entity_for(scope, depth);
      break;
   case IVL_SCT_FORK:
      error("unsupported construct (fork) at %s:%d: a named fork block has "
            "no VHDL translation", ivl_scope_def_file(scope),
            ivl_scope_def_lineno(scope));
      return 1;
   default:
      // The other scope types are expanded later on
      break;
   }

   ++depth;
   int rc = ivl_scope_children(scope, draw_skeleton_scope, NULL);
   --depth;
   return rc;
}

extern "C" int draw_all_signals(ivl_scope_t scope, void *)
{
   if (!is_default_scope_instance(scope))
      return 0;  // Not interested in this instance

   if (ivl_scope_type(scope) == IVL_SCT_MODULE) {
      vhdl_entity *ent = find_entity(scope);
      assert(ent);

      declare_signals(ent, scope);
   }
   else if (ivl_scope_type(scope) == IVL_SCT_GENERATE) {
      // Because generate scopes don't appear in the
      // output VHDL all their signals are added to the
      // containing entity (after being uniqued)

      ivl_scope_t parent = ivl_scope_parent(scope);
      while (ivl_scope_type(parent) == IVL_SCT_GENERATE)
         parent = ivl_scope_parent(parent);

      vhdl_entity* ent = find_entity(parent);
      assert(ent);

      declare_signals(ent, scope);
   }

   return ivl_scope_children(scope, draw_all_signals, scope);
}

/*
 * Draw all tasks and functions in the hierarchy.
 */
extern "C" int draw_functions(ivl_scope_t scope, void *_parent)
{
   if (!is_default_scope_instance(scope))
      return 0;  // Not interested in this instance

   // A SystemVerilog class's methods have no translation; a call to one is
   // reported where it is made (translate_ufunc, draw_utask)
   if (ivl_scope_type(scope) == IVL_SCT_CLASS)
      return 0;

   ivl_scope_t parent = static_cast<ivl_scope_t>(_parent);
   if (ivl_scope_type(scope) == IVL_SCT_FUNCTION) {
      if (draw_function(scope, parent) != 0)
         return 1;
   }
   else if (ivl_scope_type(scope) == IVL_SCT_TASK) {
      if (draw_task(scope, parent) != 0)
         return 1;
   }

   return ivl_scope_children(scope, draw_functions, scope);
}

/*
 * Make concurrent assignments for constants in nets. This works
 * bottom-up so that the driver is in the lowest instance it can.
 * This also has the side effect of generating all the necessary
 * nexus code.
 */
extern "C" int draw_constant_drivers(ivl_scope_t scope, void *)
{
   if (!is_default_scope_instance(scope))
      return 0;  // Not interested in this instance

   ivl_scope_children(scope, draw_constant_drivers, scope);

   if (ivl_scope_type(scope) == IVL_SCT_MODULE
       || ivl_scope_type(scope) == IVL_SCT_GENERATE) {
      // For generate scopes, hoist drivers to the containing module entity
      // (mirroring declare_signals/draw_all_signals).
      ivl_scope_t ent_scope = scope;
      while (ivl_scope_type(ent_scope) == IVL_SCT_GENERATE)
         ent_scope = ivl_scope_parent(ent_scope);
      vhdl_entity *ent = find_entity(ent_scope);
      assert(ent);

      int nsigs = ivl_scope_sigs(scope);
      for (int i = 0; i < nsigs; i++) {
         ivl_signal_t sig = ivl_scope_sig(scope, i);

         // j is the canonical word (pin) index, 0 .. count-1, whatever the
         // array's Verilog base (it started at the base, which skipped every
         // word of an array whose base is not 0)
         for (unsigned j = 0;
              j < ivl_signal_array_count(sig);
              j++) {
            // Make sure the nexus code is generated
            ivl_nexus_t nex = ivl_signal_nex(sig, j);
            if (!nex) continue;  // skip virtual pins
            seen_nexus(nex);

            nexus_private_t *priv =
               static_cast<nexus_private_t*>(ivl_nexus_get_private(nex));
            assert(priv);

            vhdl_scope *arch_scope = ent->get_arch()->get_scope();

            // A tri1/tri0 pull sits on the first plain signal of the net,
            // bottom-up (so in the lowest scope that has one). On a net
            // with no other driver it replaces the strong constant.
            if (priv->pull != 0 && j == 0
                && ivl_signal_port(sig) == IVL_SIP_NONE
                && ivl_signal_dimensions(sig) == 0) {
               emit_tri_pull(ent->get_arch(),
                             nexus_to_var_ref(arch_scope, nex), priv->pull);
               priv->pull = 0;
               if (priv->tri_default) {
                  priv->const_driver = NULL;
                  priv->tri_default = false;
               }
            }

            // Don't drive inputs -- except one the module drives itself
            // (declared inout): the constant is then one of its drivers --
            // nor an inout port VHDL declares `in' (one only read)
            const vhdl_port_decl *cpd = dynamic_cast<const vhdl_port_decl*>(
               ent->get_scope()->get_decl(get_renamed_signal(sig)));
            if (priv->const_driver
                && (ivl_signal_port(sig) != IVL_SIP_INPUT
                    || g_driven_inputs.count(sig))
                && (cpd == NULL || cpd->get_mode() != VHDL_PORT_IN)) {
               // nexus_to_var_ref selects the word of an unpacked array
               // (`m(j) <= const'), so every word's constant driver is
               // drawn, not only word 0's.
               vhdl_var_ref *ref = nexus_to_var_ref(arch_scope, nex);

               // Scalar constant with a strength spec (assign
               // (pull1, pull0) w = 1'b1): drive through the strength
               // buffer so resolution sees the specified level instead
               // of a full-strength assignment.  Opposing constant
               // drivers on the same net each get their own driver.
               list<const_drv_t> all;
               const_drv_t first = { priv->const_driver,
                                     priv->const_drive0,
                                     priv->const_drive1,
                                     priv->const_bits };
               all.push_back(first);
               all.splice(all.end(), priv->const_extra);
               // If ANY constant driver has a strength spec, route ALL
               // of them through strength buffers: the kernel solver
               // then owns the whole resolution.  A remaining plain
               // driver would re-resolve against the exported view in
               // the two-level l3d alphabet, where supply-vs-strong
               // collapses to X (drive_strength su1st0)
               bool any_nonstrong = false;
               for (list<const_drv_t>::iterator it = all.begin();
                    it != all.end(); ++it)
                  if (it->drive0 != IVL_DR_STRONG
                      || it->drive1 != IVL_DR_STRONG)
                     any_nonstrong = true;
               const unsigned sig_w = ivl_signal_width(sig);
               int cd_n = 0;
               for (list<const_drv_t>::iterator it = all.begin();
                    it != all.end(); ++it, ++cd_n) {
                  vhdl_var_ref *dref =
                     cd_n == 0 ? ref : nexus_to_var_ref(arch_scope, nex);
                  if (get_sv2vhdl_mode() && sig_w == 1
                      && any_nonstrong) {
                     ostringstream bs;
                     bs << "cd" << cd_n << "_" << ivl_signal_basename(sig);
                     emit_strength_buf(ent->get_arch(), dref, it->expr,
                                       it->drive1, it->drive0,
                                       bs.str().c_str());
                  }
                  else if (get_sv2vhdl_mode() && sig_w > 1
                           && any_nonstrong
                           && it->bits.length() == sig_w
                           && dref->get_type() != NULL
                           && dref->get_type()->get_name()
                              == VHDL_TYPE_LOGIC3D_VECTOR) {
                     // Vector constant with a strength spec: one
                     // strength buffer per bit so each bit's kernel
                     // net resolves at the specified level and the
                     // str1/str0 selection happens per bit value
                     // (multi_bit_strength).  bits[b] is canonical
                     // LSB-first, matching the (b) slice.
                     for (unsigned b = 0; b < sig_w; b++) {
                        vhdl_var_ref *bref =
                           nexus_to_var_ref(arch_scope, nex);
                        bref->set_slice(new vhdl_const_int(b));
                        ostringstream bs;
                        bs << "cd" << cd_n << "b" << b << "_"
                           << ivl_signal_basename(sig);
                        emit_strength_buf(ent->get_arch(), bref,
                                          new vhdl_const_bit(it->bits[b]),
                                          it->drive1, it->drive0,
                                          bs.str().c_str());
                     }
                  }
                  else {
                     // A real constant is also the signal's initial
                     // value: until the assignment's first delta the
                     // signal would read 0.0, and `r / 2.0' would
                     // divide by zero at time 0 (a fatal error in VHDL)
                     const vhdl_const_real *cr =
                        dynamic_cast<const vhdl_const_real*>(it->expr);
                     vhdl_signal_decl *sd = dynamic_cast<vhdl_signal_decl*>(
                        arch_scope->get_decl(dref->get_name()));
                     if (cr != NULL && sd != NULL && !sd->has_initial()
                         && dref->get_slice() == NULL)
                        sd->set_initial(new vhdl_const_real(cr->get_value()));
                     ent->get_arch()->add_stmt
                        (new vhdl_cassign_stmt(dref, it->expr));
                  }
               }
               priv->const_driver = NULL;
            }

            // Connect up any signals which are wired together in the
            // same nexus
            scope_nexus_t *sn = visible_nexus(priv, arch_scope);

            // Make sure we don't drive inputs
            if (ivl_signal_port(sn->sig) != IVL_SIP_INPUT) {
               for (list<ivl_signal_t>::const_iterator it = sn->connect.begin();
                    it != sn->connect.end();
                    ++it) {
                  const vhdl_type* rtype = vhdl_type_for_signal(sn->sig);
                  const vhdl_type* ltype = vhdl_type_for_signal(*it);

                  if (priv->has_inout) {
                     // Bidirectional short: the two signals are the same net,
                     // joined through an inout port (e.g. module id(a,a)). A
                     // one-directional assign can't model that, so instantiate
                     // sv_alias -- the resolver then wires both endpoints
                     // together (a true wire join). Mark the instance with the
                     // origin module so the short is traceable (e.g. to `id`).
                     static int alias_seq = 0;
                     char inst[64];
                     snprintf(inst, sizeof inst, "sv_alias_%d_inst", alias_seq++);
                     vhdl_entity_inst *ai =
                        new vhdl_entity_inst(inst, "sv2vhdl", "sv_alias", "strength");
                     ai->map_port("a",
                        new vhdl_var_ref(get_renamed_signal(sn->sig).c_str(), rtype));
                     ai->map_port("b",
                        new vhdl_var_ref(get_renamed_signal(*it).c_str(), ltype));
                     ent->get_arch()->add_stmt(ai);
                     if (!priv->inout_module.empty())
                        ent->get_arch()->add_attribute_spec(
                           "nvc_alias_origin", inst, "label", priv->inout_module);
                  }
                  else {
                     vhdl_var_ref *rref =
                        new vhdl_var_ref(get_renamed_signal(sn->sig).c_str(), rtype);
                     vhdl_var_ref *lref =
                        new vhdl_var_ref(get_renamed_signal(*it).c_str(), ltype);

                     // Make sure the LHS and RHS have the same type
                     vhdl_expr* rhs = rref->cast(lref->get_type());

                     ent->get_arch()->add_stmt(new vhdl_cassign_stmt(lref, rhs));
                  }
               }
            }
            sn->connect.clear();
         }
      }
   }

   return 0;
}

extern "C" int draw_all_logic_and_lpm(ivl_scope_t scope, void *)
{
   if (!is_default_scope_instance(scope))
      return 0;  // Not interested in this instance

   if (ivl_scope_type(scope) == IVL_SCT_MODULE) {
      vhdl_entity *ent = find_entity(scope);
      assert(ent);

      set_active_entity(ent);
      {
         declare_logic(ent->get_arch(), scope);
         declare_lpm(ent->get_arch(), scope);
         draw_switches(ent->get_arch(), scope);
      }
      set_active_entity(NULL);
   }
   else if (ivl_scope_type(scope) == IVL_SCT_GENERATE) {
      // Generate-block logic/LPM belongs to the enclosing module's
      // entity; walk up the scope chain (past nested generates) and
      // draw into its arch with the same active-entity context that
      // the surrounding module set up.
      ivl_scope_t mod = ivl_scope_parent(scope);
      while (mod && ivl_scope_type(mod) == IVL_SCT_GENERATE)
         mod = ivl_scope_parent(mod);
      if (mod && ivl_scope_type(mod) == IVL_SCT_MODULE) {
         vhdl_entity *ent = find_entity(mod);
         if (ent) {
            set_active_entity(ent);
            {
               declare_logic(ent->get_arch(), scope);
               declare_lpm(ent->get_arch(), scope);
               draw_switches(ent->get_arch(), scope);
            }
            set_active_entity(NULL);
         }
      }
   }

   return ivl_scope_children(scope, draw_all_logic_and_lpm, scope);
}

static string lowercase(const string &s)
{
   string l(s);
   for (string::size_type i = 0; i < l.size(); i++)
      l[i] = tolower((unsigned char)l[i]);
   return l;
}

/*
 * The statement labels in use in an architecture, lowercase (VHDL is case
 * insensitive). A label is declared in the architecture's declarative
 * region, so no two statements may share one. Seeded from the statements
 * present when the first module instance is placed (the gate and switch
 * instances, drawn earlier), then kept up to date by draw_hierarchy.
 */
static map<const vhdl_arch*, set<string> > g_arch_labels;

static set<string> &arch_labels(vhdl_arch *arch)
{
   map<const vhdl_arch*, set<string> >::iterator it = g_arch_labels.find(arch);
   if (it != g_arch_labels.end())
      return it->second;
   set<string> &labels = g_arch_labels[arch];
   const conc_stmt_list_t &stmts = arch->get_stmts();
   for (conc_stmt_list_t::const_iterator s = stmts.begin();
        s != stmts.end(); ++s) {
      if (const vhdl_comp_inst *ci = dynamic_cast<const vhdl_comp_inst*>(*s))
         labels.insert(lowercase(ci->get_inst_name()));
      else if (const vhdl_entity_inst *ei =
                  dynamic_cast<const vhdl_entity_inst*>(*s))
         labels.insert(lowercase(ei->get_inst_name()));
      else if (const vhdl_process *p = dynamic_cast<const vhdl_process*>(*s)) {
         if (!p->get_label().empty())
            labels.insert(lowercase(p->get_label()));
      }
   }
   return labels;
}

/*
 * The label of a module instance statement, made unique in the parent
 * architecture. The genvar suffix alone does not separate two generate blocks
 * that name their instances alike (`begin : ga leaf u ...' and `begin : gb
 * leaf u ...' both give u_g0): on a clash the enclosing generate block names
 * are prefixed (gb_u_g0), and if that is taken too, a number is appended.
 * The `-- Verilog instance:' comment keeps the Verilog path either way.
 */
static string unique_instance_label(vhdl_arch *arch, ivl_scope_t scope,
                                    const string &label)
{
   set<string> &labels = arch_labels(arch);
   const vhdl_scope *ascope = arch->get_scope();
   string cand = label;
   if (labels.count(lowercase(cand))) {
      string pfx;
      for (ivl_scope_t p = ivl_scope_parent(scope);
           p != NULL && ivl_scope_type(p) == IVL_SCT_GENERATE;
           p = ivl_scope_parent(p)) {
         string b = ivl_scope_basename(p);
         string::size_type br = b.find('[');
         if (br != string::npos)
            b.erase(br);
         pfx = b + "_" + pfx;
      }
      string base = pfx + label;
      replace_consecutive_underscores(base);
      if (base[0] == '_')
         base = "inst" + base;
      cand = base;
      for (int k = 2; labels.count(lowercase(cand))
              || ascope->name_collides(cand)
              || find_entity(cand) != NULL
              || is_vhdl_reserved_word(cand); k++) {
         ostringstream ss;
         ss << base << "_" << k;
         cand = ss.str();
      }
   }
   labels.insert(lowercase(cand));
   return cand;
}

// Verilog hierarchical name of a module instance relative to the module
// that contains it: the enclosing generate scopes (named, or genblk<n> as
// iverilog names unnamed ones) and the instance basename, '.'-joined.
static string verilog_relative_path(ivl_scope_t scope)
{
   string path = ivl_scope_basename(scope);
   for (ivl_scope_t p = ivl_scope_parent(scope);
        p != NULL && ivl_scope_type(p) == IVL_SCT_GENERATE;
        p = ivl_scope_parent(p))
      path = string(ivl_scope_basename(p)) + "." + path;
   return path;
}

extern "C" int draw_hierarchy(ivl_scope_t scope, void *_parent)
{
   if (ivl_scope_type(scope) == IVL_SCT_MODULE && _parent) {
      ivl_scope_t parent = static_cast<ivl_scope_t>(_parent);

      // Skip over any containing generate scopes
      while (ivl_scope_type(parent) == IVL_SCT_GENERATE)
         parent = ivl_scope_parent(parent);

      if (!is_default_scope_instance(parent))
         return 0;  // Not generating code for the parent instance so
                    // don't generate for the child

      vhdl_entity *ent = find_entity(scope);
      if (!ent) return 0;  // No VHDL entity for this scope (e.g. generate block)

      vhdl_entity *parent_ent = find_entity(parent);
      if (!parent_ent) return 0;  // No VHDL entity for parent scope

      vhdl_arch *parent_arch = parent_ent->get_arch();
      if (!parent_arch) return 0;

      // Create a forward declaration for it
      const vhdl_scope *parent_scope = parent_arch->get_scope();
      if (!parent_scope->have_declared(ent->get_name())) {
         vhdl_decl *comp_decl = vhdl_component_decl::component_decl_for(ent);
         parent_arch->get_scope()->add_decl(comp_decl);
      }

      // And an instantiation statement
      string inst_name = ivl_scope_basename(scope);
      inst_name += genvar_unique_suffix(ivl_scope_parent(scope));
      if (inst_name == ent->get_name()
          || parent_scope->name_collides(inst_name)
          || find_entity(inst_name) != NULL
          || is_vhdl_reserved_word(inst_name)) {

         // Would produce an invalid instance name
         inst_name += "_inst";
      }

      // Need to replace any [ and ] characters that result
      // from generate statements
      string::size_type loc = inst_name.find('[', 0);
      if (loc != string::npos)
         inst_name.erase(loc, 1);

      loc = inst_name.find(']', 0);
      if (loc != string::npos)
         inst_name.erase(loc, 1);

      // An escaped instance name (\a+b , \x.y , \9lives ) may hold any
      // character: the label keeps its letters, digits and underscores, and
      // starts with a letter (the Verilog instance comment keeps the name)
      for (string::size_type k = 0; k < inst_name.size(); k++)
         if (!isalnum((unsigned char)inst_name[k]) && inst_name[k] != '_')
            inst_name[k] = '_';
      if (!inst_name.empty() && isdigit((unsigned char)inst_name[0]))
         inst_name = "inst_" + inst_name;

      // No leading or trailing underscores
      if (inst_name[0] == '_')
         inst_name = "inst" + inst_name;
      if (*inst_name.rbegin() == '_')
         inst_name += "inst";

      // Can't have two consecutive underscores
      replace_consecutive_underscores(inst_name);

      // Make sure the name doesn't collide with anything we've
      // already declared
      avoid_name_collision(inst_name, parent_arch->get_scope());

      // ... nor with another statement's label
      inst_name = unique_instance_label(parent_arch, scope, inst_name);

      // Record the finalized label for icg2en guard-path emission
      icg2en_note_label(scope, inst_name);

      vhdl_comp_inst *inst =
         new vhdl_comp_inst(inst_name.c_str(), ent->get_name().c_str());
      port_map(scope, parent_ent, inst);

      // The second comment line gives the instance's Verilog hierarchical
      // name relative to the enclosing module: the generate scopes between
      // the module and the instance, then the instance itself, with generate
      // and instance-array indices as Verilog writes them (g[1].xb, ua[0]).
      // The VHDL label cannot be mapped back (genvar suffixes, [] removed,
      // collision renames), so this comment is what tools read.
      ostringstream ss;
      ss << "Generated from instantiation at "
         << ivl_scope_file(scope) << ":" << ivl_scope_lineno(scope)
         << "\nVerilog instance: " << verilog_relative_path(scope);
      inst->set_comment(ss.str());

      parent_arch->add_stmt(inst);
   }

   return ivl_scope_children(scope, draw_hierarchy, scope);
}

int draw_scope(ivl_scope_t scope, void *_parent)
{
   int rc = draw_skeleton_scope(scope, _parent);
   if (rc != 0)
      return rc;

   rc = draw_all_signals(scope, _parent);
   if (rc != 0)
      return rc;

   rc = draw_all_logic_and_lpm(scope, _parent);
   if (rc != 0)
      return rc;

   rc = draw_hierarchy(scope, _parent);
   if (rc != 0)
      return rc;

   rc = draw_functions(scope, _parent);
   if (rc != 0)
      return rc;

   rc = draw_constant_drivers(scope, _parent);
   if (rc != 0)
      return rc;
   return 0;
}
