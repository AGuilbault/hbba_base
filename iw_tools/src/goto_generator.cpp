/// \file goto_generator.cpp A GoTo desire generator demo motivation.

#include <iw_tools/common.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <hbba_msgs/AddDesires.h>
#include <hbba_msgs/RemoveDesires.h>
#include <iw_tools/events_filter.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/action_definition.h>
#include <ros/ros.h>

namespace iw_tools 
{
    /// \brief A SimpleActionServer for move base goals.
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>
            GoToServer;

    /// \brief A demo motivation class that generates GoTo desires.
    ///
    /// This class is meant as a motivation sample.
    /// It takes stamped poses as input and generate a corresponding desire.
    /// It also serves an action server named "goto".
    /// The desire is removed when the goal is achieved (monitored by HBBA
    /// events).
    ///
    /// Action:
    ///  - goto:        A move_base_msgs/MoveBaseAction server.
    /// Topics:
    ///  - goto_goal:   A geometry_msgs/PoseStamped input.
    ///  - hbba_events: HBBA events input.
    ///
    class GoToGenerator
    {
    private:
        ros::Subscriber                 sub_goal_;
        ros::ServiceClient              scl_add_desires_;
        ros::ServiceClient              scl_rem_desires_;
        boost::scoped_ptr<EventsFilter> filter_;

        GoToServer                      goto_server_;

        hbba_msgs::AddDesires           req_add_desires_;
        std::vector<hbba_msgs::Desire>& desires_set_;
        hbba_msgs::RemoveDesires        req_rem_desires_;
        std::vector<std::string>&       desires_ids_;

        enum {
            DES_GOTO = 0,
            DES_SIZE = 1
        };

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for main topics and services.
        /// \param np Node handle for private topics, parameters.
        GoToGenerator(ros::NodeHandle& n, ros::NodeHandle& np):
            goto_server_(n,
                         "goto",
                         boost::bind(&GoToGenerator::execute, this, _1),
                         false),
            desires_set_(req_add_desires_.request.desires),
            desires_ids_(req_rem_desires_.request.ids)
        {
            sub_goal_ = n.subscribe("goto_goal", 1, &GoToGenerator::goalCB, this);

            scl_add_desires_ = n.serviceClient<hbba_msgs::AddDesires>(
                "add_desires");
            scl_rem_desires_ = n.serviceClient<hbba_msgs::RemoveDesires>(
                "remove_desires");

            desires_set_.resize(DES_SIZE);
            hbba_msgs::Desire& goto_d = desires_set_[DES_GOTO];
            np.param<std::string>("id", goto_d.id, "goto_gen_goal");
            goto_d.type      = "GoTo"; 
            goto_d.utility   = 1;
            int intensity;
            np.param<int>("intensity", intensity, 1);
            goto_d.intensity = intensity;
            goto_d.security  = false;    
            // NOTE: Params will be filled in goal callback.
            
            desires_ids_.resize(DES_SIZE);
            desires_ids_[DES_GOTO] = goto_d.id; 

            filter_.reset(new EventsFilter(n, goto_d.id));
            filter_->allEventsCB(&GoToGenerator::eventsCB, this);

            goto_server_.start();

        }

    private:
        void goalCB(const geometry_msgs::PoseStamped& msg)
        {
            // From common.hpp:
            poseStampedToNavGoal(msg, desires_set_[DES_GOTO].params); 

            scl_add_desires_.call(req_add_desires_);
        }

        void eventsCB(const hbba_msgs::Event& evt)
        {
            if (evt.type == hbba_msgs::Event::ACC_ON) {
                scl_rem_desires_.call(req_rem_desires_);
                goto_server_.setSucceeded();
            }
        }

        void execute(const move_base_msgs::MoveBaseGoalConstPtr& goal)
        {
            goalCB(goal->target_pose);
            
        }

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goto_generator");

    ros::NodeHandle n, np("~");
    iw_tools::GoToGenerator node(n, np);

    ros::spin();

    return 0;
}


