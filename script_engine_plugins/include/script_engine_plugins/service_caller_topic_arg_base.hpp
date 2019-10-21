#ifndef SERVICE_CALLER_TOPIC_ARG_BASE_HPP
#define SERVICE_CALLER_TOPIC_ARG_BASE_HPP

#include "engine_module.hpp"
#include <boost/function.hpp>
#include <string>
#include <ros/ros.h>
#include <tr1/unordered_map>

namespace script_engine_plugins
{
	typedef std::tr1::unordered_map<std::string, ros::ServiceClient> service_client_map_t;

	service_client_map_t service_client_map_;
	/// \brief A template class for service calling engine module.
	///
	/// For a usage sample, see eval_srv_test.cpp.
	template<
		class T, 		// Service type
		const char FName[],	// Script function name
		// Argument conversion function
		void AFun(const v8::FunctionCallbackInfo<v8::Value>&, typename T::Request&),
		// Result conversion function
		v8::Handle<v8::Value> RFun(const typename T::Response&)>
	class service_caller_topic_arg_base: public engine_module
	{
	public:
		/// \brief Constructor.
		service_caller_topic_arg_base()
		{
		}

		virtual void init(v8::Handle<v8::ObjectTemplate>& global)
		{
			using namespace v8;
			global->Set(String::NewFromUtf8(Isolate::GetCurrent(),FName),
				FunctionTemplate::New(Isolate::GetCurrent(),&this_t::call));
		}

	private:
		static void call(const v8::FunctionCallbackInfo<v8::Value>& args)
		{
			ros::NodeHandle n;

			v8::String::Utf8Value v8_service_name(args[0]);
			const char* service_name = *v8_service_name;

			ROS_DEBUG("Calling %s service from script_engine", service_name);

			ros::ServiceClient* service_client;
			service_client_map_t::iterator i = service_client_map_.find(service_name);
			if(i == service_client_map_.end())
			{
				ros::ServiceClient nservice_client = n.serviceClient<T>(service_name);
				service_client_map_[std::string(service_name)] = nservice_client;
				i = service_client_map_.find(service_name);
			}
			service_client = &(i->second);

			typename T::Request req;
			typename T::Response res;
			AFun(args, req);
			service_client->call(req, res);
			args.GetReturnValue().Set(RFun(res));
		}

		typedef service_caller_topic_arg_base<T, FName, AFun, RFun> this_t;

	};
}

#endif

