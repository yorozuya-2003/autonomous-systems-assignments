from crewai import BaseAgent
from crewai import Message
from datetime import datetime, timedelta

class CIAgent(BaseAgent):
    def send_navigation_request(self, bi_agent_id, start_position, goal_position, visitor_id, constraints=None):
        message = Message(
            message_type="NAVIGATION_REQUEST",
            sender=self.id,  
            receiver=bi_agent_id,  
            content={
                "start_position": start_position,
                "goal_position": goal_position,
                "visitor_id": visitor_id,
                "constraints": constraints or {},
                "timestamp": int(datetime.now().timestamp())
            }
        )
        self.send(message)
    
    def receive_navigation_response(self, message):
        if message.content["status"] == "APPROVED":
            path = message.content["path"]
            estimated_time = message.content["estimated_time"]
            print(f"Path received for visitor {message.content['visitor_id']}: {path}, Estimated Time: {estimated_time}")
        else:
            reason = message.content.get("reason", "No reason provided")
            print(f"Navigation request denied: {reason}")

    def receive_oos_notification(self, message):
        
        print(f"BI Agent {message.sender} is out of service for {message.content['out_of_service_duration']}.")
        if "estimated_recovery_time" in message.content:
            recovery_time = datetime.fromtimestamp(message.content["estimated_recovery_time"])
            print(f"Estimated recovery time: {recovery_time}")


class BIAgent(BaseAgent):
    def receive_navigation_request(self, message):
        start_position = message.content["start_position"]
        goal_position = message.content["goal_position"]
        visitor_id = message.content["visitor_id"]
        constraints = message.content.get("constraints", {})
        
        path = self.calculate_path(start_position, goal_position)
        
        if path:
            response = Message(
                message_type="NAVIGATION_RESPONSE",
                sender=self.id,
                receiver=message.sender,  
                content={
                    "visitor_id": visitor_id,
                    "path": path,
                    "estimated_time": len(path) * 2,  
                    "status": "APPROVED",
                    "timestamp": int(datetime.now().timestamp())
                }
            )
        else:
            response = Message(
                message_type="NAVIGATION_RESPONSE",
                sender=self.id,
                receiver=message.sender,
                content={
                    "visitor_id": visitor_id,
                    "status": "DENIED",
                    "reason": "Path calculation failed",
                    "timestamp": int(datetime.now().timestamp())
                }
            )
        self.send(response)
    
    def send_oos_notification(self, ci_agent_id, duration):
        notification = Message(
            message_type="OOS_NOTIFICATION",
            sender=self.id,
            receiver=ci_agent_id,
            content={
                "status": "OUT_OF_SERVICE",
                "out_of_service_duration": duration,
                "timestamp": int(datetime.now().timestamp()),
                "estimated_recovery_time": int((datetime.now() + timedelta(hours=duration)).timestamp())
            }
        )
        self.send(notification)
    
    def calculate_path(self, start_position, goal_position):
        if start_position and goal_position:
            return [start_position, goal_position]  
        return None
