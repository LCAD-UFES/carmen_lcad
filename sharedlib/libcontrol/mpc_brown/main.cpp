#include <cstdio>
#include <cstdlib>
#include <string>
#include "vehicles.h"
#include "trajectories.h"
#include "model_predictive_control.h"
#include "mpc_brown_messages.h"
#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/mpc_brown_interface.h>


using namespace std;


static void
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(CARMEN_MPC_BROWN_NAME, IPC_VARIABLE_LENGTH, CARMEN_MPC_BROWN_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MPC_BROWN_NAME);
}

void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("MPC BROWN: disconnected.\n");
        exit(0);
    }
}

int main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);


    // define the messages
    register_ipc_messages();

    //map <string, double> vehicle_constants = function_X1();
    //ENTRADA DO PROGRAMA
    TrajectoryTube straight_trajectory;
    double len = 30.;
    double val = 5.;
    straight_trajectory.t.push_back(0.);
    straight_trajectory.t.push_back(len / val);
    straight_trajectory.s.push_back(0.);
    straight_trajectory.s.push_back(len);
    straight_trajectory.V.push_back(val);
    straight_trajectory.V.push_back(val);
    straight_trajectory.A.push_back(0.);
    straight_trajectory.A.push_back(0.);
    straight_trajectory.E.push_back(0.);
    straight_trajectory.E.push_back(0.);
    straight_trajectory.N.push_back(0.);
    straight_trajectory.N.push_back(30.);
    straight_trajectory.head.push_back(0.);
    straight_trajectory.head.push_back(0.);
    straight_trajectory.curv.push_back(0.);
    straight_trajectory.curv.push_back(0.);
    TrajectoryTrackingMPC X1CMPC;
    X1CMPC.current_state.E.push_back(0.);
    X1CMPC.current_state.N.push_back(0.);
    X1CMPC.current_state.phi.push_back(0.);
    X1CMPC.current_state.Ux.push_back(5.);
    X1CMPC.current_state.Uy.push_back(0.);
    X1CMPC.current_state.r.push_back(0.);
    X1CMPC.current_control.delta_ = (0.);
    X1CMPC.current_control.Fxf = 0.;
    X1CMPC.current_control.Fxr = (0.);
    MPCTimeSteps time_steps = make_MPCTimeSteps(5, 10, 0.01, 0.2, true);
    X1CMPC.time_steps = time_steps;
    while(1)
    { 
            
    /*
        compute_time_steps!(X1DMPC, 0.)
        compute_linearization_nodes!(X1DMPC)
        update_QP!(X1DMPC)
        solve!(X1DMPC)

        compute_time_steps!(X1CMPC, 0.)
        compute_linearization_nodes!(X1CMPC)
        update_QP!(X1CMPC)
        solve!(X1CMPC)
    
        ////SAIDA DO PROGRAMA
    
        carmen_mpc_brown_message_t message;
        message.host = carmen_get_host();
        message.timestamp = carmen_get_time(); 
        message.delta_ = X1CMPC.current_control.delta_;
        message.Fxf = X1CMPC.current_control.Fxf;
        message.Fxr = X1CMPC.current_control.Fxr;

        carmen_mpc_brown_publish_message(&message);       
    */
    }
    

    return 0;
}