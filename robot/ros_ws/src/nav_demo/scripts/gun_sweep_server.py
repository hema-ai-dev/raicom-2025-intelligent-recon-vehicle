#!/usr/bin/env python3
"""ROS service for the deterministic sweep strategy used in competition.

The node defaults to a disabled state. Set ``~armed:=true`` only in a
controlled test area after verifying the serial protocol and emergency stop.
"""

import serial
import rospy

from nav_demo.srv import GunSweep, GunSweepResponse


class GunSweepController:
    def __init__(self) -> None:
        self.port = rospy.get_param("~port", "/dev/gun")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.sweep_step = max(1, int(rospy.get_param("~sweep_step", 1)))
        self.shot_duration = max(0.0, float(rospy.get_param("~shot_duration", 0.2)))
        self.armed = bool(rospy.get_param("~armed", False))
        self.serial = None

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo("opened gun controller %s at %d baud", self.port, self.baudrate)
        except serial.SerialException as exc:
            rospy.logerr("cannot open gun controller %s: %s", self.port, exc)

        rospy.on_shutdown(self.safe_stop)
        self.service = rospy.Service("/gun_sweep", GunSweep, self.sweep_callback)

    def write_packet(self, device_id: int, value: int) -> bool:
        if self.serial is None or not self.serial.is_open:
            return False
        if not 0 <= value <= 255:
            rospy.logerr("command value outside byte range: %d", value)
            return False
        try:
            self.serial.write(bytes([0xAA, 0xBB, device_id, value, 0x0D, 0x0A]))
            return True
        except serial.SerialException as exc:
            rospy.logerr("serial write failed: %s", exc)
            return False

    def set_servo(self, servo_id: int, angle: int) -> bool:
        if not 0 <= angle <= 180:
            return False
        return self.write_packet(servo_id, angle)

    def set_fire(self, enabled: bool) -> bool:
        if enabled and not self.armed:
            rospy.logwarn("fire request rejected because ~armed is false")
            return False
        return self.write_packet(0x03, 0x01 if enabled else 0x00)

    def safe_stop(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.write_packet(0x03, 0x00)
            self.serial.close()

    @staticmethod
    def validate_request(request):
        if not 0 <= request.min_angle <= request.max_angle <= 180:
            return "angles must satisfy 0 <= min_angle <= max_angle <= 180"
        if not 0 <= request.angle <= 180:
            return "vertical angle must be between 0 and 180"
        if not 1 <= request.num <= 10:
            return "num must be between 1 and 10"
        return None

    def sweep_callback(self, request) -> GunSweepResponse:
        error = self.validate_request(request)
        if error:
            return GunSweepResponse(success=False, message=error)
        if self.serial is None or not self.serial.is_open:
            return GunSweepResponse(success=False, message="serial controller is unavailable")
        if not self.armed:
            return GunSweepResponse(success=False, message="controller is not armed")

        try:
            if not self.set_servo(0x02, request.angle):
                return GunSweepResponse(success=False, message="vertical servo command failed")

            for _ in range(request.num):
                for angle in range(request.min_angle, request.max_angle + 1, self.sweep_step):
                    if not self.set_servo(0x01, angle):
                        return GunSweepResponse(success=False, message="horizontal servo command failed")
                    if not self.set_fire(True):
                        return GunSweepResponse(success=False, message="fire command failed")
                    rospy.sleep(self.shot_duration)

            return GunSweepResponse(
                success=True,
                message="sweep completed ({} -> {})".format(request.min_angle, request.max_angle),
            )
        except Exception as exc:
            rospy.logerr("sweep failed: %s", exc)
            return GunSweepResponse(success=False, message=str(exc))
        finally:
            self.set_fire(False)
            self.set_servo(0x01, request.min_angle)


def main() -> None:
    rospy.init_node("gun_sweep_controller")
    GunSweepController()
    rospy.loginfo("gun sweep service is ready; armed=%s", rospy.get_param("~armed", False))
    rospy.spin()


if __name__ == "__main__":
    main()
