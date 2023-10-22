import rospy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.msg import CollisionObject

if __name__ == '__main__':
    rospy.init_node('get_object_poses', anonymous=True)

    # Hizmeti çağırmak için bir istemci oluştur
    rospy.wait_for_service('/get_planning_scene')
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

    # İstek oluştur
    request = PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
    response = get_planning_scene(request)

    # planning_scene içindeki bütün objelerin pozisyonlarını al
    for collision_object in response.scene.world.collision_objects:
        if collision_object.id.startswith("Box_"):
            print(f"Object Name: {collision_object.id}")
            print(f"Object Pose: {collision_object.primitive_poses[0]}\n")
