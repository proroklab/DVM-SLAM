from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from typing import Tuple
import time


class InteractiveMarkerWrapper():
    def __init__(self, name: str, position: Tuple[float, float], marker_server, menu_handler):
        self.name = name
        self.position = position
        self.marker_server = marker_server
        self.menu_handler = menu_handler

        self.create_marker()

    def create_marker(self):
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "world"
        interactive_marker.name = self.name
        interactive_marker.pose.position.x = float(self.position[0])
        interactive_marker.pose.position.y = float(self.position[1])
        interactive_marker.pose.position.z = -0.5
        interactive_marker.scale = 0.2

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.always_visible = True

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        control.markers.append(marker)
        interactive_marker.controls.append(control)

        self.marker_server.insert(
            interactive_marker, feedback_callback=self.marker_feedback)
        self.menu_handler.apply(self.marker_server, interactive_marker.name)

        self.marker_server.applyChanges()

    def marker_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            self.position = (feedback.pose.position.x,
                             feedback.pose.position.y)
