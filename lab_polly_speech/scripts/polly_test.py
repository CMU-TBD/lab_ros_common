import rospy
from lab_common.msg import(
    speakGoal,
    speakAction
)
import actionlib


def main():
    polly_client = actionlib.SimpleActionClient("lab_polly_speech/speak", speakAction)
    polly_client.wait_for_server()

    goal = speakGoal()
    goal.text = "this is polly test node"
    polly_client.send_goal_and_wait(goal)

if __name__ == '__main__':
    rospy.init_node("polly_node_test")
    main()
