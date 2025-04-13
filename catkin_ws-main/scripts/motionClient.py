#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Pose2D
from tsr1.srv import GetGo2point

def get2point_client(param): 
    rospy.loginfo("Esperando la respuesta del servicio...")
    rospy.wait_for_service('/get2point')
    try: 
        get2poin = rospy.ServiceProxy('/get2point', GetGo2point)
        srv_response = get2poin(param)
        print("Debo verme si el servico esta completo")
        return srv_response

    except rospy.ServiceException as e: 
        print("Falló petición del servicio: %s"%e)
        


if __name__ == "__main__":
    param = Pose2D()
    param.x = 5
    param.y = 5
    param.theta = 0
    srv = get2point_client(param)
    print ("%s"%(srv))
    rospy.logerr("Peticion concluida")


