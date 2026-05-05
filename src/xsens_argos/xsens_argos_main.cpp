#include <iostream>
#include <string>
#include <signal.h>

// --- Unitree SDK 2 ---
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

// --- CARMEN ---
extern "C" {
    #include <carmen/carmen.h>
    #include <carmen/xsens_messages.h>
    #include <carmen/xsens_mtig_messages.h>
    #include <carmen/xsens_mtig_interface.h>
}

using namespace unitree::robot;

// Variáveis Globais (Padrão Skeleton)
static carmen_xsens_global_quat_message xsens_quat_msg;
static int xsens_type = 1; // 1 = Simular MTi-G (IMU + GPS/Vel)
static int print_xsens = 1;

// --- MÉTODOS DE FORMATAÇÃO E PUBLICAÇÃO (IGUAL AO XSENS_MTIG) ---

void 
publish_mti_quat_message(carmen_xsens_global_quat_message message)
{
    IPC_RETURN_TYPE err;
    err = IPC_publishData(CARMEN_XSENS_GLOBAL_QUAT_NAME, &message);
    carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}

carmen_xsens_global_quat_message
make_xsens_mti_quat_message(const unitree_go::msg::dds_::LowState_* low_state)
{
    carmen_xsens_global_quat_message msg;

    // Acceleration
    msg.m_acc.x = low_state->imu_state().accelerometer()[0];
    msg.m_acc.y = low_state->imu_state().accelerometer()[1];
    msg.m_acc.z = low_state->imu_state().accelerometer()[2];

    // Gyro
    msg.m_gyr.x = low_state->imu_state().gyroscope()[0];
    msg.m_gyr.y = low_state->imu_state().gyroscope()[1];
    msg.m_gyr.z = low_state->imu_state().gyroscope()[2];

    // Magnetism (O Go2 não envia mag puro no LowState, zeramos ou simulamos)
    msg.m_mag.x = 0.0;
    msg.m_mag.y = 0.0;
    msg.m_mag.z = 0.0;

    // Quaternion
    msg.quat_data.m_data[0] = low_state->imu_state().quaternion()[0]; // w
    msg.quat_data.m_data[1] = low_state->imu_state().quaternion()[1]; // x
    msg.quat_data.m_data[2] = low_state->imu_state().quaternion()[2]; // y
    msg.quat_data.m_data[3] = low_state->imu_state().quaternion()[3]; // z

    // Temperature and Count[cite: 1]
    msg.m_temp = low_state->imu_state().temperature();
    msg.m_count = 0.0;

    msg.timestamp = carmen_get_time();
    msg.host = carmen_get_host();

    return msg;
}

// --- CALLBACK E PARÂMETROS ---

void LowStateCallback(const void* message) 
{
    const auto* low_state = static_cast<const unitree_go::msg::dds_::LowState_*>(message);

    printf("Oi! \n");

    // Método: Cria a mensagem e publica[cite: 1]
    xsens_quat_msg = make_xsens_mti_quat_message(low_state);
    publish_mti_quat_message(xsens_quat_msg);

    // Print no padrão de log XSENS_QUAT solicitado[cite: 1]
    static double last_print_time = 0;
    if (print_xsens)
    {
        double new_print_time = carmen_get_time();
        printf("XSENS_QUAT %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %lf %s\n",
               xsens_quat_msg.m_acc.x, xsens_quat_msg.m_acc.y, xsens_quat_msg.m_acc.z,
               xsens_quat_msg.m_gyr.x, xsens_quat_msg.m_gyr.y, xsens_quat_msg.m_gyr.z,
               xsens_quat_msg.m_mag.x, xsens_quat_msg.m_mag.y, xsens_quat_msg.m_mag.z,
               xsens_quat_msg.quat_data.m_data[0], xsens_quat_msg.quat_data.m_data[1], 
               xsens_quat_msg.quat_data.m_data[2], xsens_quat_msg.quat_data.m_data[3],
               xsens_quat_msg.m_temp, (int)xsens_quat_msg.m_count, 
               xsens_quat_msg.timestamp, xsens_quat_msg.host);
        double freq = 1.0/(new_print_time - last_print_time);
        printf("freq: %lf\n", freq);
        last_print_time = new_print_time;
    }

}

static int 
read_parameters(int argc, char **argv)
{
    carmen_param_t param_list[] = {
        {(char *) "xsens", (char *) "type", CARMEN_PARAM_INT, &xsens_type, 0, NULL}
    };
    carmen_param_allow_unfound_variables(0);
    carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

    carmen_param_t optional_param_list[] = {
        {(char *) "commandline", (char *) "print_xsens", CARMEN_PARAM_ONOFF, &print_xsens, 0, NULL},
    };
    carmen_param_allow_unfound_variables(1);
    carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list)/sizeof(optional_param_list[0]));

    return 0;
}

void define_ipc_messages(void)
{
    IPC_RETURN_TYPE err;
    err = IPC_defineMsg(CARMEN_XSENS_MTIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_MTIG_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_MTIG_NAME);

    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);
}

void shutdown_module(int signo)
{
    if(signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("\nxsens_argos: disconnected.\n");
        exit(0);
    }
}

int main(int argc, char **argv) 
{
    // Inicialização CARMEN[cite: 1]
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);
    read_parameters(argc, argv);
    define_ipc_messages();

    signal(SIGINT, shutdown_module);

    // Argumentos de linha de comando
    std::string interface = "wlan0";
    int domain_id = 0;
    // if (argc > 1) interface = argv[1];
    // if (argc > 2) domain_id = std::stoi(argv[2]);

    printf("[xsens_argos] Iniciando: Interface=%s, Domain=%d\n", 
           interface.c_str(), domain_id);

    // Inicialização Unitree
    ChannelFactory::Instance()->Init(domain_id, interface.c_str());
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
    lowstate_subscriber->InitChannel(LowStateCallback, 1);

    carmen_ipc_dispatch();
    return 0;
}