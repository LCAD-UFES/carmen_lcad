#include "stehs_planner.hpp"

// constructor
StehsPlanner::StehsPlanner():
        start(),
        goal(),
        distance_map(nullptr),
        goal_list_message(nullptr),
        robot_config(),
        active(false),
        show_debug_info(),
        cheat(),
        lane_ready(false),
        distance_map_ready(false),
        goal_ready(false),
        circle_path(),
        state_list() {

    // creates a new opencv window
    cv::namedWindow("CirclePath", cv::WINDOW_AUTOSIZE);

    std::cout<<"OpenCV Version used:"<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<std::endl;

}

// destructor
StehsPlanner::~StehsPlanner() {

    cv::destroyAllWindows();

}

double
StehsPlanner::Distance(double ax, double ay, double bx, double by)
{
    double dx = ax - bx;
    double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}


double
StehsPlanner::Distance(const State &a, const State &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double distance = sqrt(dx * dx + dy * dy);

    return (distance);
}


double
StehsPlanner::Distance2(const State &a, const State &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double distance = (dx * dx + dy * dy);

    return (distance);
}


carmen_point_t
traj_to_point_t(State traj)
{
    carmen_point_t point;

    point.x = traj.x;
    point.y = traj.y;
    point.theta = traj.theta;

    return (point);
}

double
StehsPlanner::ObstacleDistance(double x, double y)
{
    carmen_point_t p;

    p.x = x;
    p.y = y;

    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}

double
StehsPlanner::ObstacleDistance(const State &point)
{
    carmen_point_t p;
    p = traj_to_point_t(point);

    return (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&p, distance_map));
}


std::list<CircleNode>
StehsPlanner::BuildCirclePath(CircleNodePtr node)
{
    std::list<CircleNode> temp_circle_path;

    while (node != NULL)
    {
        temp_circle_path.push_front(*node);

        node = node->parent;
    }

    return(temp_circle_path);
}

bool
StehsPlanner::Exist(CircleNodePtr current, std::vector<CircleNodePtr> &closed_set)
{
    std::vector<CircleNodePtr>::iterator it = closed_set.begin();
    std::vector<CircleNodePtr>::iterator end = closed_set.end();

    while (it != end)
    {
        if (current->parent != (*it) && current->parent != (*it)->parent && current->circle.Overlap((*it)->circle, MAX_OVERLAP_FACTOR))
            return true;

        it++;
    }

    return false;
}


void
StehsPlanner::Expand(CircleNodePtr current,	std::priority_queue<CircleNodePtr, std::vector<CircleNodePtr>, CircleNodePtrComparator> &open_set)
{

    // Considering a 2m radius parent circle that involves the car we
    // want 32 children circles
    unsigned int children_amount = (unsigned int) (16.0*current->circle.radius);

    double theta = 2.0*M_PI / (double) children_amount;
    double c = std::cos(theta);
    double s = std::sin(theta);
    double t;

    // the parent coordinates
    double px = current->circle.x;
    double py = current->circle.y;

    // the parent cost
    double pg = current->g;

    // the parent radius
    double pr = current->circle.radius;

    //we start at angle = 0
    double x = pr;
    double y = 0.0;

    for(unsigned int i = 0; i < children_amount; i++)
    {
        double nx = x + px;
        double ny = y + py;

        double obstacle_distance = ObstacleDistance(nx, ny);

        if (obstacle_distance > robot_config.width * 0.5) {

            // create the next child
            // TODO verificar se é possível remover filhos com overlap
            open_set.push(new CircleNode(nx, ny, obstacle_distance, pg + pr, pg + pr + Distance(nx, ny, goal.x, goal.y), current));

        }

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
}


std::list<CircleNode>
StehsPlanner::SpaceExploration(CircleNodePtr start_node, CircleNodePtr goal_node)
{
    std::list<CircleNode> temp_circle_path;

    start_node->g = 0.0;
    start_node->f = Distance(start_node->circle.x, start_node->circle.y, goal_node->circle.x, goal_node->circle.y);

    goal_node->g = goal_node->f = DBL_MAX;

    // create the priority queue
    std::priority_queue<CircleNodePtr, std::vector<CircleNodePtr>, CircleNodePtrComparator> open_set;

    std::vector<CircleNodePtr> closed_set;

    Expand(start_node, open_set);

    while (!open_set.empty())
    {
        // get the circle wich is the closest to the goal node
        CircleNodePtr current = open_set.top();
        open_set.pop();

        if (goal_node->f < current->f)
        {
            temp_circle_path = BuildCirclePath(goal_node->parent);

            while(!open_set.empty())
            {
                CircleNodePtr tmp = open_set.top();

                open_set.pop();

                delete tmp;
            }
            break;
        }
        else if (!Exist(current, closed_set))
        {
            Expand(current, open_set);

            if (current->circle.Overlap(goal_node->circle, MIN_OVERLAP_FACTOR))
            {

                if (current->f < goal_node->g)
                {
                    goal_node->g = current->f;
                    goal_node->parent = current;
                }
            }
            closed_set.push_back(current);
        }
        else
        {
            delete current;
        }
    }

    while(!closed_set.empty())
    {
        CircleNodePtr tmp = closed_set.back();

        closed_set.pop_back();

        delete tmp;
    }

    return (temp_circle_path);
}


int
StehsPlanner::FindClosestRDDFIndex()
{
    double current_distance = 0.0;
    double previous_distance = Distance(start, goal_list_message->poses[0]);
    int i;

    for (i = 1; i < goal_list_message->number_of_poses; i++)
    {
        current_distance = Distance(start, goal_list_message->poses[i]);

        if (current_distance > previous_distance)
            break;
    }

    return (i - 1);
}


int
StehsPlanner::FindNextRDDFIndex(double radius_2, int current_rddf_index)
{
    int i = current_rddf_index + 1;

    while (Distance2(goal_list_message->poses[current_rddf_index], goal_list_message->poses[i]) < radius_2)
    {
        i++;
    }

    return (i - 1);
}


int
StehsPlanner::FindNextRDDFFreeIndex(int current_rddf_index)
{

    double obstacle_distance;

    do
    {
        obstacle_distance = ObstacleDistance(goal_list_message->poses[current_rddf_index++]);
    } while (obstacle_distance < robot_config.width * 0.5);

    return (current_rddf_index);
}


void
StehsPlanner::ConnectCirclePathGaps()
{
    std::list<CircleNode> temp_circle_path;

    std::list<CircleNode>::iterator it = circle_path.begin();
    std::list<CircleNode>::iterator previous_it = it;
    ++it;

    while (it != circle_path.end())
    {
        if(!it->circle.Overlap(previous_it->circle, MIN_OVERLAP_FACTOR))
        {
            temp_circle_path = SpaceExploration(&(*previous_it), &(*it));
            if(!temp_circle_path.empty())
                circle_path.splice(previous_it,temp_circle_path);
            else
            {
                // TODO O que fazer?
            }
        }
        previous_it = it;
        ++it;
    }
}


void
StehsPlanner::UpdateCircleGoalDistance()  // Iterate the entire circle path computing the accumulated distance to goal
{
    std::list<CircleNode>::reverse_iterator next = circle_path.rbegin();
    std::list<CircleNode>::reverse_iterator current = next;
    std::list<CircleNode>::reverse_iterator end = circle_path.rend();

    current++;
    next->f = 0.0;

    for (; current != end; current++, next++)
    {
        current->f = Distance(current->circle.x, current->circle.y, next->circle.x, next->circle.y) + next->f;
    }
}


void
StehsPlanner::RDDFSpaceExploration()
{
    // clear the old circle path
    circle_path.clear();

    //TODO verificar se os dados para os calculos estao disponiveis, se nao, nao fazer nada
    // create the start circle node
    CircleNodePtr start_node =  new CircleNode(start.x, start.y, ObstacleDistance(start), 0.0, 0.0, nullptr);

    // create the goal circle node
    CircleNodePtr goal_node = new CircleNode(goal.x, goal.y, ObstacleDistance(goal), DBL_MAX, DBL_MAX, nullptr);

    int current_rddf_index = FindClosestRDDFIndex();

    CircleNodePtr current_node = new CircleNode(goal_list_message->poses[current_rddf_index].x, goal_list_message->poses[current_rddf_index].y, ObstacleDistance(goal_list_message->poses[current_rddf_index]), DBL_MAX, DBL_MAX, nullptr);

    circle_path.push_back(*start_node);

//	if (current_node->circle.Overlap(start_node->circle, 0.5))
//	{
//		circle_path.push_back(*start_node);
//		delete current_node;
//		current_node = start_node;
//	}
//	else
//	{
//		SpaceExploration(start_node, current_node);
//	}

    double obstacle_distance = 0.0;

    while (!current_node->circle.Overlap(goal_node->circle, MIN_OVERLAP_FACTOR))
    {
        // We decided to use radius^2 to optimize the Distance computation
        current_rddf_index = FindNextRDDFIndex(current_node->circle.radius * current_node->circle.radius, current_rddf_index);

        obstacle_distance = ObstacleDistance(goal_list_message->poses[current_rddf_index]);
        if (obstacle_distance > robot_config.width * 0.5)
        {
            current_node = new CircleNode(goal_list_message->poses[current_rddf_index].x, goal_list_message->poses[current_rddf_index].y, obstacle_distance, 0.0, 0.0, nullptr);
        }
        else
        {
            current_rddf_index = FindNextRDDFFreeIndex(current_rddf_index);
            obstacle_distance = ObstacleDistance(goal_list_message->poses[current_rddf_index]);

            current_node = new CircleNode(goal_list_message->poses[current_rddf_index].x, goal_list_message->poses[current_rddf_index].y, obstacle_distance, 0.0, 0.0, nullptr);
            //SpaceExploration(current_node, next_free_node);
        }
        circle_path.push_back(*current_node);
    }

    circle_path.push_back(*goal_node);

    ConnectCirclePathGaps();

    UpdateCircleGoalDistance();


/*
    std::list<CircleNode>::iterator it;
    for(it = circle_path.begin(); it != circle_path.end(); ++it)
    {
        printf("x: %f y: %f r: %f\n",it->circle.x, it->circle.y, it->circle.radius);
    }
*/


}


std::list<CircleNode>::iterator
StehsPlanner::FindNearestCircle(const State &state)
{
	std::list<CircleNode>::iterator it = circle_path.begin();
	std::list<CircleNode>::iterator end = circle_path.end();

	double min_distance = DBL_MAX;
	double current_distance;

	for (; it != end; it++)
	{
		current_distance = Distance(state.x, state.y, it->circle.x, it->circle.y);

		if (current_distance < min_distance)
		{
			min_distance = current_distance;
		}
		else
		{
			it--;
			break;
		}
	}
	return it;
}


double
StehsPlanner::TimeHeuristic(const State &state)
{
    std::list<CircleNode>::iterator nearest = FindNearestCircle(state);
    std::list<CircleNode>::iterator previous = nearest;
    previous--;
    std::list<CircleNode>::iterator next = nearest;
    next++;

    double nearest_circle_distance, previous_circle_distance = DBL_MAX, next_circle_distance = DBL_MAX, min_distance;

    // TODO Previous is needed?
    if(previous != circle_path.end())
    {
    	previous_circle_distance = Distance(state.x, state.y, previous->circle.x, previous->circle.y) + previous->f;
    }

    if(next != circle_path.end())
	{
    	next_circle_distance = Distance(state.x, state.y, next->circle.x, next->circle.y) + next->f;
	}

    nearest_circle_distance = Distance(state.x, state.y, nearest->circle.x, nearest->circle.y) + nearest->f;

    min_distance = std::min(nearest_circle_distance, std::min(next_circle_distance, previous_circle_distance));

    if (state.v == 0.0)
        return (DBL_MAX);
    else
        return ((min_distance) / fabs(state.v));
}


double
StehsPlanner::DistanceHeuristic(const State &state) // TODO Optimize this linear search verifying only the next, current and previous circles
{
    std::list<CircleNode>::iterator it = circle_path.begin();
    std::list<CircleNode>::iterator end = circle_path.end();

    double circle_distance = DBL_MAX;
    double current_distance;
    double goal_distance = DBL_MAX;

    for (; it != end; it++)
    {
        current_distance = Distance(state.x, state.y, it->circle.x, it->circle.y);

        if (current_distance < circle_distance)
        {
            circle_distance = current_distance;
            goal_distance = it->f;
        }
    }

    return ((circle_distance + goal_distance));
}


void
StehsPlanner::BuildStateList(StateNodePtr node)
{
	state_list.clear();

	carmen_ackerman_path_point_t path_point;
	while(node != nullptr)
	{
		path_point.x      = node->state.x    ;
		path_point.y      = node->state.y    ;
		path_point.theta  = node->state.theta;
		path_point.v      = node->state.v    ;
		path_point.phi    = node->state.phi  ;
		path_point.time = node->step_size;
		state_list.push_front(path_point);
		node = node->parent;
	}
}

// TODO we need to implement the circle radius clustering
bool
StehsPlanner::Exist(StateNodePtr current, std::vector<StateNodePtr> &closed_set, double k) {

    std::vector<StateNodePtr>::iterator it = closed_set.begin();
    std::vector<StateNodePtr>::iterator end = closed_set.end();

    while (it != end)
    {
        if (current->Equals(*(*it), k))
        	return true;

        it++;
    }

    return false;
}

StateNodePtr
StehsPlanner::GetNextState(StateNodePtr current_state, double a, double w, double step_size)
{
    StateNodePtr next_state = new StateNode(*current_state);
    double target_phi = current_state->state.phi + w * step_size;
    double target_v = current_state->state.v + a * step_size;
    if(target_v > goal.v)
    	target_v = goal.v;

    target_phi = carmen_clamp(-robot_config.max_phi, target_phi, robot_config.max_phi);

    double distance_traveled = 0.0;
    next_state->state = carmen_libcarmodel_recalc_pos_ackerman(current_state->state, target_v, target_phi,
            step_size, &distance_traveled, DELTA_T, robot_config);

    next_state->parent = current_state;
    next_state->g = current_state->g + step_size;
    next_state->h = TimeHeuristic(next_state->state);
//    next_state->h = Distance(next_state->state.x, next_state->state.y, goal.x, goal.y); //DistanceHeuristic(next_state->state); // TODO usar distancia direto????
    next_state->f = next_state->g + next_state->h;
    next_state->step_size = step_size;

    return (next_state);
}



double
StehsPlanner::UpdateStep(StateNodePtr state_node)   // TODO Pensar melhor nessa funcao, parece sempre retornar o MIN_STEP_SIZE
{
	CircleNodePtr nearest_circle = &(*FindNearestCircle(state_node->state)); // a função FindNearestCircle retorna um iterador, aqui pegamos o elemento.
	double v = std::fabs(state_node->state.v);

//	if (state_node->state.v != 0.0)
//	{
		//printf("F %lf\n", nearest_circle->f);
		return (std::min(std::min(ALFA * nearest_circle->circle.radius, BETA * nearest_circle->f) / v, MAX_STEP_SIZE));
//	}
//	else
//	{
//		return DELTA_T * 50000.0;
//	}
}


bool
StehsPlanner::Collision(StateNodePtr state_node)
{
	double circle_radius = (robot_config.width + 0.4) * 0.5; // metade da largura do carro + um espacco de guarda

	carmen_point_t state = traj_to_point_t(state_node->state);

	carmen_point_t trash;
	trash.x = 0.0;
	trash.y = 0.0;
	trash.theta = 0.0;

	return (carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(&state, trash, robot_config, distance_map, circle_radius) > 0.0); // Returns 0 if there is not a collision
}

cv::Mat imgem;

void
StehsPlanner::Expand(
        StateNodePtr current_state,
        std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set,
        std::vector<StateNodePtr> &closed_set,
        double k)
{
    // the car acceleration
    double a[3] = {-1.0, 0.0, 1.0};
    double w[3] = {-0.1, 0.0, 0.1}; //TODO ler velocidade angular do volante do carmen.ini

    double step_size = k * UpdateStep(current_state);

    // TODO tratar isso na UpdateStep
    if(step_size < 0.2)
    	step_size = 0.2;

    // the acceleration loop
    for (int i = 0; i < 3; ++i)
    {
        // the steering angle acceleration
        for (int j = 0; j < 3; ++j)
        {
        	//printf("\nState %lf %lf %lf %lf %lf %lf\n", current_state->state.x, current_state->state.y, current_state->state.theta, current_state->state.phi, current_state->state.v, step_size);
            // get the next state
            StateNodePtr next_state = GetNextState(current_state, a[i], w[j], step_size);

        	//printf("State %lf %lf %lf %lf %lf\n", next_state->state.x, next_state->state.y, next_state->state.theta, next_state->state.phi, next_state->state.v);

            // FIXME checar a mudança de 1.0 para k
            if (!Exist(next_state, closed_set, 1.0 /*k*/) && !Collision(next_state))
            {
                open_set.push(next_state);

                //ShowState(next_state, imgem);
            }
            else
            {
                delete next_state;
            }
        }
    }
}


void
StehsPlanner::GoalExpand(StateNodePtr current, StateNodePtr &goal_node,
		std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set)
{
	if(mrpt::math::angDistance<double>(current->state.theta, goal_node->state.theta) < MIN_THETA_DIFF )
	{
		goal_node->parent = current;
		goal_node->f = current->f;

	}

	// FIXME modificar isso;
	(void) open_set;
}


void
StehsPlanner::SetSwap(
        std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set,
        std::vector<StateNodePtr> &closed_set) {

	std::vector<StateNodePtr> outputvec(open_set.size());

	std::copy(&(open_set.top()), &(open_set.top()) + open_set.size(), &outputvec[0]);

	while(!open_set.empty())
	{
		open_set.pop();
	}

	for (unsigned int i = 0; i < closed_set.size(); i++)
	{
		open_set.push(closed_set[i]);
	}

	closed_set = outputvec;
}

void
StehsPlanner::HeuristicSearch()
{

    //StateNode(const State &s, double g_, double f_, StateNode *p);
    // build the start node, the initial configuration
    StateNodePtr start_node = new StateNode(start, 0.0, TimeHeuristic(start), nullptr);

    StateNodePtr goal_node = new StateNode(goal, DBL_MAX, DBL_MAX, nullptr);

    // create the priority queue
    std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> open_set;

    std::vector<StateNodePtr> closed_set;

    open_set.push(start_node);

    // the inital step-rate
    double k = 1.0;
    int cont = 0;

	//imgem = ShowCirclePath(closed_set);

    while (!open_set.empty())
    {
        // get the circle wich is the closest to the goal node
        StateNodePtr current = open_set.top();
        open_set.pop();

        if (goal_node->f < current->f)
        {
            BuildStateList(goal_node);

            while(!open_set.empty())
            {
                StateNodePtr tmp = open_set.top();

                open_set.pop();

                delete tmp;
            }

            //printf("Nstate %ld cont %d\n", state_list.size(), cont);
            //printf("Sucesso!!\n");
            break;

        }
        // find the children states configuration
        Expand(current, open_set, closed_set, k);
        cont += 6;

        //printf("Nstate %d\n", cont);

        //printf("h %lf f %lf T %ld\n", current->h, current->f, open_set.size());

        if (current->h < RGOAL)
        {
//            printf("Goal!!\n");

            GoalExpand(current, goal_node, open_set);
        }

        closed_set.push_back(current);

        if (open_set.empty())
        {
        	//printf("Open set empty!\n");

            k *= 0.5;

            if (k > KMIN)
            {
                SetSwap(open_set, closed_set);
            }

        }

    }

//    ShowCirclePath(closed_set);

    while(!closed_set.empty())
    {
        StateNodePtr tmp = closed_set.back();

        closed_set.pop_back();

        delete tmp;
    }

}


void
StehsPlanner::GeneratePath()
{
	//printf("Inicio space exploration\n");
	RDDFSpaceExploration();
	//printf("Fim space exploration\n");
	if (!circle_path.empty())
	{
		HeuristicSearch();
	}
	else
	{
		// TODO Pensar no que fazer. Consumir o plano, parar?
		printf("Não foi possível encontrar um caminho válido.\n");
	}

	ShowCirclePath();
}


unsigned char* StehsPlanner::GetCurrentMap() {

    unsigned int width = distance_map->config.x_size;
    unsigned int height = distance_map->config.y_size;

    unsigned int size = width * height;

    unsigned char *map = new unsigned char[size];

    for (unsigned int i = 0; i < size; ++i) {

        // get the current row
        unsigned int row = (height - 1) - i % height;

        // get the current col
        unsigned int col = i / height;

        // get the current index
        unsigned int index = row * width + col;

        if (0.0 == distance_map->complete_x_offset[i] && 0.0 == distance_map->complete_y_offset[i]) {

            map[index] = 0;

        } else {

            map[index] = 255;

        }

    }

    return map;
}

cv::Mat
StehsPlanner::ShowCirclePath(std::vector<StateNodePtr> &state_node)
{

	//printf("Entrou\n");
	// get the current map
    unsigned char *map = GetCurrentMap();

    unsigned int width = distance_map->config.x_size;
    unsigned int height = distance_map->config.y_size;

    // create a new opencv image
    cv::Mat img(width, height, CV_8UC1, map);

    // get the inverse resolution
    double inverse_resolution = 1.0 / distance_map->config.resolution;

    //
    std::list<CircleNode>::iterator it = circle_path.begin();
    std::list<CircleNode>::iterator end = circle_path.end();

    while (it != end) {

        unsigned int row = height - std::floor((it->circle.y - distance_map->config.y_origin) * inverse_resolution + 0.5);
        unsigned int col = std::floor((it->circle.x - distance_map->config.x_origin) * inverse_resolution + 0.5);

        double radius = it->circle.radius * inverse_resolution;

        // void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1)
        cv::circle(img, cv::Point(col, row), radius, cv::Scalar(0, 0, 0), 1);

        // show the current image
//        cv::imshow("CirclePath", img);

//        cv::waitKey(30);

        // move to next circle
        ++it;

    }

    std::vector<StateNodePtr>::iterator its = state_node.begin();
    std::vector<StateNodePtr>::iterator ends = state_node.end();

    while (its != ends)
    {
    	unsigned int row = height - std::floor(((*its)->state.y - distance_map->config.y_origin) * inverse_resolution + 0.5);
    	unsigned int col = std::floor(((*its)->state.x - distance_map->config.x_origin) * inverse_resolution + 0.5);

    	cv::rectangle(img, cv::Point (col-3, row-3), cv::Point (col+3, row+3), cv::Scalar(0,0,0));

//    	cv::imshow("CirclePath", img);
//
//    	cv::waitKey(30);

    	its++;
    }

//    cv::imshow("CirclePath", img);
//    cv::waitKey(1);

	//printf("Saiu\n");
    delete [] map;

    return img;
}


cv::Mat
StehsPlanner::ShowState(StateNodePtr &state_node, cv::Mat img)
{
	unsigned int height = distance_map->config.y_size;
	double inverse_resolution = 1.0 / distance_map->config.resolution;

	unsigned int row = height - std::floor((state_node->state.y - distance_map->config.y_origin) * inverse_resolution + 0.5);
	unsigned int col = std::floor((state_node->state.x - distance_map->config.x_origin) * inverse_resolution + 0.5);

	cv::rectangle(img, cv::Point (col-3, row-3), cv::Point (col+3, row+3), cv::Scalar(0,0,0));

    cv::imshow("CirclePath", img);
    cv::waitKey(1);

    return img;
}


void StehsPlanner::ShowCirclePath() {

	//printf("Entrou\n");
	// get the current map
    unsigned char *map = GetCurrentMap();

    unsigned int width = distance_map->config.x_size;
    unsigned int height = distance_map->config.y_size;

    // create a new opencv image
    cv::Mat img(width, height, CV_8UC1, map);

    // get the inverse resolution
    double inverse_resolution = 1.0 / distance_map->config.resolution;

    //
    std::list<CircleNode>::iterator it = circle_path.begin();
    std::list<CircleNode>::iterator end = circle_path.end();

    while (it != end) {

        unsigned int row = height - std::floor((it->circle.y - distance_map->config.y_origin) * inverse_resolution + 0.5);
        unsigned int col = std::floor((it->circle.x - distance_map->config.x_origin) * inverse_resolution + 0.5);

        double radius = it->circle.radius * inverse_resolution;

        // void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1)
        cv::circle(img, cv::Point(col, row), radius, cv::Scalar(0, 0, 0), 1);

        // show the current image
//        cv::imshow("CirclePath", img);

//        cv::waitKey(30);

        // move to next circle
        ++it;

    }

    std::list<carmen_ackerman_path_point_t>::iterator its = state_list.begin();
    std::list<carmen_ackerman_path_point_t>::iterator ends = state_list.end();

    while (its != ends)
    {
    	unsigned int row = height - std::floor((its->y - distance_map->config.y_origin) * inverse_resolution + 0.5);
    	unsigned int col = std::floor((its->x - distance_map->config.x_origin) * inverse_resolution + 0.5);

    	cv::rectangle(img, cv::Point (col-3, row-3), cv::Point (col+3, row+3), cv::Scalar(0,0,0));

//    	cv::imshow("CirclePath", img);
//
//    	cv::waitKey(30);

    	its++;
    }

    cv::imshow("CirclePath", img);
    cv::waitKey(1);

	//printf("Saiu\n");
    delete [] map;
}
