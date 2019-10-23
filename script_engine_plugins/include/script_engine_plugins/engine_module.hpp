#ifndef ENGINE_MODULE_HPP
#define ENGINE_MODULE_HPP

#include <v8.h>
#include <pluginlib/class_loader.h>

namespace script_engine_plugins
{
	/// \brief A plugin module for the V8 execution engine.
	class engine_module
	{
	public:
		engine_module() {};
		virtual ~engine_module() {};

		/// \brief Called by the script engine once ready.
		///
		/// In this method, you should register functions and objects in the
		/// global object. 
		virtual void init(v8::Isolate* isolate, v8::Handle<v8::ObjectTemplate>& global) = 0;

	};

}

#endif

