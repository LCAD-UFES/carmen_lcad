#!/usr/bin/env python3
import math
import rospy
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage


def tf_callback(msg):
    # O tópico /tf envia um array de transformadas a cada mensagem
    for transform in msg.transforms:

        # Limpamos as barras (/) para evitar problemas de compatibilidade
        parent = transform.header.frame_id.strip('/')
        child = transform.child_frame_id.strip('/')

        # Verifica se é exatamente a TF que queremos
        if parent == 'odom' and child == 'base_link':

            t = transform.transform.translation
            r = transform.transform.rotation

            # Converte o Quaternion (x, y, z, w) para ângulos de Euler (Roll, Pitch, Yaw) em radianos
            quaternion = [r.x, r.y, r.z, r.w]
            roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)

            # Converte radianos para graus
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)

            # Timestamp:
            timestamp = transform.header.stamp.to_sec()

            # PRINTANDO
            print(f"TF_ODOM_BASELINK X_Y_Z {t.x} {t.y} {t.z} R_P_Y {roll_rad} {pitch_rad} {yaw_rad} TIMESTAMP {timestamp}")

            # # Formata a saída no terminal
            # print('\n--- Nova TF recebida (odom -> base_link) ---')
            # print(
            #     f'Tempo: {transform.header.stamp.secs}.{transform.header.stamp.nsecs}'
            # )
            # print(f'Posição (x, y, z) [m]:   {t.x:.3f}, {t.y:.3f}, {t.z:.3f}')
            # print(
            #     f'Rotação RPY [graus]:      Roll: {roll_deg:.2f}°, Pitch:'
            #     f' {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°'
            # )


def main():
    # Inicializa o nó no ROS 1
    rospy.init_node('tf_direct_listener', anonymous=True)

    # Inscreve diretamente no tópico /tf
    rospy.Subscriber('/tf', TFMessage, tf_callback, queue_size=100)

    rospy.loginfo("Escutando as transformadas de 'odom' para 'base_link'...")

    # Mantém o script rodando e escutando callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass