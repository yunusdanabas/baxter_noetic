#!/usr/bin/python3

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
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

from __future__ import print_function
import argparse
import sys

import rospy

from baxter_interface import (
    DigitalIO,
    Gripper,
    Navigator,
    CHECK_VERSION,
)


class GripperConnect(object):
    """
    Connects wrist button presses to gripper open/close commands.

    Uses the DigitalIO Signal feature to make callbacks to connected
    action functions when the button values change.
    """

    def __init__(self, arm, lights=True):
        """
        @type arm: str
        @param arm: arm of gripper to control {left, right}
        @type lights: bool
        @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm
        # inputs
        self._close_io = DigitalIO('%s_upper_button' % (arm,))  # 'dash' btn
        self._open_io = DigitalIO('%s_lower_button' % (arm,))   # 'circle' btn
        self._light_io = DigitalIO('%s_lower_cuff' % (arm,))    # cuff squeeze
        # outputs
        self._gripper = Gripper('%s' % (arm,), CHECK_VERSION)
        self._nav = Navigator('%s' % (arm,))

        # connect callback fns to signals
        if self._gripper.type() != 'custom':
            if not (self._gripper.calibrated() or
                    self._gripper.calibrate() == True):
                rospy.logwarn("%s (%s) calibration failed.",
                              self._gripper.name.capitalize(),
                              self._gripper.type())
        else:
            msg = (("%s (%s) not capable of gripper commands."
                   " Running cuff-light connection only.") %
                   (self._gripper.name.capitalize(), self._gripper.type()))
            rospy.logwarn(msg)

        self._gripper.on_type_changed.connect(self._check_calibration)
        self._open_io.state_changed.connect(self._open_action)
        self._close_io.state_changed.connect(self._close_action)

        if lights:
            self._light_io.state_changed.connect(self._light_action)

        rospy.loginfo("%s Cuff Control initialized...",
                      self._gripper.name.capitalize())

    def _open_action(self, value):
        if value and self._is_grippable():
            rospy.logdebug("gripper open triggered")
            self._gripper.open()

    def _close_action(self, value):
        if value and self._is_grippable():
            rospy.logdebug("gripper close triggered")
            self._gripper.close()

    def _light_action(self, value):
        if value:
            rospy.logdebug("cuff grasp triggered")
        else:
            rospy.logdebug("cuff release triggered")
        self._nav.inner_led = value
        self._nav.outer_led = value

    def _check_calibration(self, value):
        if self._gripper.calibrated():
            return True
        elif value == 'electric':
            rospy.loginfo("calibrating %s...",
                          self._gripper.name.capitalize())
            return (self._gripper.calibrate() == True)
        else:
            return False

    def _is_grippable(self):
        return (self._gripper.calibrated() and self._gripper.ready())


def main():
    """RSDK Gripper Button Control Example

    Connects cuff buttons to gripper open/close commands:
        'Circle' Button    - open gripper
        'Dash' Button      - close gripper
        Cuff 'Squeeze'     - turn on Nav lights

    Run this example in the background or in another terminal
    to be able to easily control the grippers by hand while
    using the robot. Can be run in parallel with other code.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-g', '--gripper', dest='gripper', default='both',
                        choices=['both', 'left', 'right'],
                        help='gripper limb to control (default: both)')
    parser.add_argument('-n', '--no-lights', dest='lights',
                        action='store_false',
                        help='do not trigger lights on cuff grasp')
    parser.add_argument('-v', '--verbose', dest='verbosity',
                        action='store_const', const=rospy.DEBUG,
                        default=rospy.INFO,
                        help='print debug statements')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_gripper_cuff_control_%s' % (args.gripper,),
                    log_level=args.verbosity)

    arms = (args.gripper,) if args.gripper != 'both' else ('left', 'right')
    grip_ctrls = [GripperConnect(arm, args.lights) for arm in arms]

    print("Press cuff buttons to control grippers. Spinning...")
    rospy.spin()
    print("Gripper Button Control Finished.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
