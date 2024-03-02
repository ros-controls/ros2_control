# Copyright 2021 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
        except ValueError:
            combo.setCurrentIndex(-1)

        # Re-populate items
        combo.blockSignals(True)  # No need to notify these changes
        combo.clear()
        combo.insertItems(0, new_vals)
        combo.setCurrentIndex(selected_id)  # Restore selection
        combo.blockSignals(False)


def _is_permutation(a, b):
    """
    Return true if a is a permutation of b, false otherwise.

    @type a []
    @type b []
    @return True if C{a} is a permutation of C{b}, false otherwise
    @rtype bool
    """
    return set(a) == set(b)
