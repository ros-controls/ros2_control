#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

def update_combo(combo, new_vals):
    """
    Update the contents of a combo box with a set of new values.

    If the previously selected element is still present in the new values, it
    will remain as active selection, even if its index has changed. This will
    not trigger any signal.

    If the previously selected element is no longer present in the new values,
    the combo will unset its selection. This will trigger signals for changed
    element.
    """
    selected_val = combo.currentText()
    old_vals = [combo.itemText(i) for i in range(combo.count())]

    # Check if combo items changed
    if not _is_permutation(old_vals, new_vals):
        # Determine if selected value is in the new list
        selected_id = -1
        try:
            selected_id = new_vals.index(selected_val)
        except (ValueError):
            combo.setCurrentIndex(-1)

        # Re-populate items
        combo.blockSignals(True)  # No need to notify these changes
        combo.clear()
        combo.insertItems(0, new_vals)
        combo.setCurrentIndex(selected_id)  # Restore selection
        combo.blockSignals(False)

def _is_permutation(a, b):
    """
    @type a []
    @type b []
    @return True if C{a} is a permutation of C{b}, false otherwise
    @rtype bool
    """
    return len(a) == len(b) and sorted(a) == sorted(b)
