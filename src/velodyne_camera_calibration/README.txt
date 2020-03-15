Para rodar o velodyne_camera_calibration, use
 ./velodyne_camera_calibration <camera number>

O programa abre uma tela que mostra a imagem da camera e os raios do Velodyne superpostos.
Ele responde aas seguintes teclas:

    if (k == 'q') camera_pose.position.x += step_size;
    if (k == 'a') camera_pose.position.x -= step_size;
    if (k == 'w') camera_pose.position.y += step_size;
    if (k == 's') camera_pose.position.y -= step_size;
    if (k == 'e') camera_pose.position.z += step_size;
    if (k == 'd') camera_pose.position.z -= step_size;

    if (k == 'r') camera_pose.orientation.roll += carmen_degrees_to_radians(angular_step_size);
    if (k == 'f') camera_pose.orientation.roll -= carmen_degrees_to_radians(angular_step_size);
    if (k == 't') camera_pose.orientation.pitch += carmen_degrees_to_radians(angular_step_size);
    if (k == 'g') camera_pose.orientation.pitch -= carmen_degrees_to_radians(angular_step_size);
    if (k == 'y') camera_pose.orientation.yaw += carmen_degrees_to_radians(angular_step_size);
    if (k == 'h') camera_pose.orientation.yaw -= carmen_degrees_to_radians(angular_step_size);

    if (k == 'u') camera_parameters.fx_factor += step_size;
    if (k == 'j') camera_parameters.fx_factor -= step_size;

    if (k == 'i') camera_parameters.fy_factor += step_size;
    if (k == 'k') camera_parameters.fy_factor -= step_size;

    if(k == 'z') step_size = step_size / 10.0;
    if(k == 'x') step_size = step_size * 10.0;

    if(k == 'n') angular_step_size = angular_step_size / 10.0;
    if(k == 'm') angular_step_size = angular_step_size * 10.0;

Use-as ara ajustar os raios do Velodyne com pontos caracteristicos da imagem do mundo.
Aa medida que os parametros vao sendo modificados, a CAM_POSE, o fx e o fy vao sendo impressos na tela.

Quando estiver satisfeito com a CAM_POSE, o fx e o fy, atualize-os no carmen ini.
