import rospy
import dynamic_reconfigure.client


class SimpleDynamicToggle:
    def __init__(self, topic_name, config_key="enabled", **kwargs):
        self.topic_name = topic_name
        self.enabled = len(self.topic_name) > 0
        rospy.loginfo("Dynamic toggle (%s) enabled: %s" % (self.topic_name, self.enabled))
        
        self.state = None
        if self.enabled:
            self.dyn_client = dynamic_reconfigure.client.Client(
                self.topic_name,
                config_callback=self.callback,
                **kwargs
            )
        else:
            self.dyn_client = None
        self.config_key = config_key

    def callback(self, config):
        if not self.enabled:
            return 
        self.state = config[self.config_key]
        rospy.loginfo("Dynamic toggle (%s) callback: %s" % (self.topic_name, self.state))
    
    def set_state(self, state):
        if not self.enabled:
            return False
        if state == self.state:
            return False
        self.dyn_client.update_configuration({self.config_key: bool(state)})
        return True

    def get_state(self):
        return self.state
