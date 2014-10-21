/*
 * gnd_lssmap_particle_evaluator_config.hpp
 */

#ifndef GND_LSSMAP_PARTICLE_EVALUTOR_CONFIG_HPP_
#define GND_LSSMAP_PARTICLE_EVALUTOR_CONFIG_HPP_

#include <string.h>

#include "gnd/gnd-util.h"
#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-lib-error.h"

// ---> type declaration
namespace gnd {
	namespace lssmap_particle_evaluator {
		struct node_config;
		typedef struct node_config node_config;

		typedef gnd::conf::parameter_array<char, 256> param_string_t;
		typedef gnd::conf::parameter_array<double, 2> param_range_t;
		typedef gnd::conf::param_int param_int_t;
		typedef gnd::conf::param_long param_long_t;
		typedef gnd::conf::param_double param_double_t;
		typedef gnd::conf::param_bool param_bool_t;
	}
} // <--- type declaration


// ---> const variables definition
namespace gnd {
	namespace lssmap_particle_evaluator {

		// ---> map file option
		static const param_string_t Default_bmp_map = {
				"bmp-map-file",
				"",
				"bmp map file path"
		};

		static const param_string_t Default_bmp_map_origin = {
				"bmp-map-origin-file",
				"",
				"bmp map origin file path"
		};
		// <--- map file option

		// ---> ros communication
		static const param_string_t Default_node_name = {
				"node-name",
				"lssmap_particle_evaluator",
				"ros-node name"
		};

		static const param_string_t Default_topic_name_particles = {
				"topic-particles",
				"localization_particles",
				"particles topic name, (subscribe, type:gnd_particle_localiser/msg_pose2darray_stamped)"
		};

		static const param_string_t Default_topic_name_particle_weight = {
				"topic-particle-weights",
				"localization_particles/weights",
				"particle weights topic name, (publish, gnd_particle_localiser/msg_localization_particle_weight)"
		};

		static const param_string_t Default_topic_name_pointcloud = {
				"topic-point-cloud",
				"point_cloud",
				"point-cloud topic name, [note] this node deal the point cloud data that is 2d laser-scanner data and it's coordinate origin should be robot position (subscribe, type:sensor_msgs::PointCloud)"
		};
		// <--- ros communication

		// ---> operating option
		static const param_double_t Default_period = {
				"period",
				gnd_msec2sec(1000),
				"period time [sec]"
		};

		static const param_range_t Default_points_select_condition_height_range = {
				"points-select-condition-height-range",
				{ 0.0, -1.0},
				"height range to select points that use particles evaluation. #1 range lower, #2 range upper. [note] if lower > upper, this item is ignored."
		};

		static const param_double_t Default_points_select_condition_ignore_horizontal_range_upper = {
				"points-select-condition-ignore-horizontal-range-upper",
				-1,
				"horizontal distance range to select points that use particles evaluation. if a point is farther than this value, it is ignored. [note] if this value < 0, this item is ignored."
		};

		static const param_double_t Default_points_select_condition_ignore_horizontal_range_lower = {
				"points-select-condition-ignore-horizontal-range-lower",
				0.02,
				"horizontal distance range to select points that use particles evaluation. if a point is nearer than this value, it is ignored. [note] if this value < 0, this item is ignored."
		};

		static const param_double_t Default_points_select_condition_culling_distance = {
				"points-select-condition-culling-distance",
				gnd_cm2m(5),
				"if contiguous points are nearer than this value, these point is ignored. [note] if this value < 0, this item is ignored."
		};
		// <--- operating option


		// ---> debug option
		static const param_double_t Default_period_cui_status_display = {
				"period-status-display",
				0.0,
				"period to display the node status in standard output [msec]. [note] it need ansi color code. [note] if this value is less than or equal 0, don't display"
		};
		// <--- debug option
	}
}
// <--- const variables definition



// ---> function declaration
namespace gnd {
	namespace lssmap_particle_evaluator {
		/**
		 * @brief initialize configure to default parameter
		 * @param [out] p : node_config
		 */
		int init_node_config(node_config *conf);


		/**
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		int fread_node_config( const char* fname, node_config *dest );
		/**
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		int get_node_config( node_config *dest, gnd::conf::configuration *src );


		/**
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		int fwrite_node_config( const char* fname, node_config *src );
		/**
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		int set_node_config( gnd::conf::configuration *dest, node_config *src );
	}
} // <--- function declaration



// ---> type definition
namespace gnd {
	namespace lssmap_particle_evaluator {
		/**
		 * @brief configuration parameter for gnd_urg_proxy node
		 */
		struct node_config {
			node_config();

			// map file option
			param_string_t bmp_map;														///< map file (bitmap)
			param_string_t bmp_map_origin;												///< map origin file
			// ros communication option
			param_string_t node_name;													///< ros-node name
			param_string_t topic_name_particles;										///< particles topic name
			param_string_t topic_name_particle_weights;									///< particle weights topic name
			param_string_t topic_name_pointcloud;										///< pointcloud topic name
			// operating option
			param_double_t period;														///< cycle to resampling
			param_range_t  points_select_condition_height_range;						///< point select condition
			param_double_t points_select_condition_ignore_horizontal_range_upper;		///< point select condition
			param_double_t points_select_condition_ignore_horizontal_range_lower;		///< point select condition
			param_double_t points_select_condition_culling_distance;					///< point select condition

			// debug option
			param_double_t period_cui_status_display;									///< no device mode for debug
		};


		inline
		node_config::node_config() {
			init_node_config(this);
		}
	}
}
// <--- type definition



// ---> function definition
namespace gnd {
	namespace lssmap_particle_evaluator {
		/*
		 * @brief initialize configuration parameter
		 * @param [out] p : node_config
		 */
		inline
		int init_node_config( node_config *p ){
			gnd_assert(!p, -1, "invalid null pointer argument\n" );

			// map file option
			memcpy( &p->bmp_map,												&Default_bmp_map,												sizeof(Default_bmp_map) );
			memcpy( &p->bmp_map_origin,											&Default_bmp_map_origin,										sizeof(Default_bmp_map_origin) );
			// ros communication option
			memcpy( &p->node_name,												&Default_node_name,												sizeof(Default_node_name) );
			memcpy( &p->topic_name_particles,									&Default_topic_name_particles,									sizeof(Default_topic_name_particles) );
			memcpy( &p->topic_name_particle_weights,							&Default_topic_name_particle_weight,							sizeof(Default_topic_name_particle_weight) );
			memcpy( &p->topic_name_pointcloud,									&Default_topic_name_pointcloud,									sizeof(Default_topic_name_pointcloud) );
			// operating option
			memcpy( &p->period,													&Default_period,												sizeof(Default_period) );
			memcpy( &p->points_select_condition_height_range,					&Default_points_select_condition_height_range,					sizeof(Default_points_select_condition_height_range) );
			memcpy( &p->points_select_condition_ignore_horizontal_range_lower,	&Default_points_select_condition_ignore_horizontal_range_lower,	sizeof(Default_points_select_condition_ignore_horizontal_range_lower) );
			memcpy( &p->points_select_condition_ignore_horizontal_range_upper,	&Default_points_select_condition_ignore_horizontal_range_upper,	sizeof(Default_points_select_condition_ignore_horizontal_range_upper) );
			memcpy( &p->points_select_condition_culling_distance,				&Default_points_select_condition_culling_distance,				sizeof(Default_points_select_condition_culling_distance) );
			// debug option
			memcpy( &p->period_cui_status_display,								&Default_period_cui_status_display,								sizeof(Default_period_cui_status_display) );
			return 0;
		}

		/*
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int fread_node_config( const char* fname, node_config *dest ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(fname)) < 0 )    return ret;

				return get_node_config(dest, &fs);
			} // <--- operation
		}
		/*
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		inline
		int get_node_config( node_config *dest, gnd::conf::configuration *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// map file option
			gnd::conf::get_parameter( src, &dest->bmp_map );
			gnd::conf::get_parameter( src, &dest->bmp_map_origin );
			// ros communication option
			gnd::conf::get_parameter( src, &dest->node_name );
			gnd::conf::get_parameter( src, &dest->topic_name_particles );
			gnd::conf::get_parameter( src, &dest->topic_name_particle_weights );
			gnd::conf::get_parameter( src, &dest->topic_name_pointcloud );
			// operating option
			gnd::conf::get_parameter( src, &dest->period );
			gnd::conf::get_parameter( src, &dest->points_select_condition_height_range );
			gnd::conf::get_parameter( src, &dest->points_select_condition_ignore_horizontal_range_lower );
			gnd::conf::get_parameter( src, &dest->points_select_condition_ignore_horizontal_range_upper );
			gnd::conf::get_parameter( src, &dest->points_select_condition_culling_distance );
			// debug option
			gnd::conf::get_parameter( src, &dest->period_cui_status_display );

			return 0;
		}



		/*
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		inline
		int fwrite_node_config( const char* fname, node_config *src ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );
			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

				return fs.write(fname);
			} // <--- operation
		}

		/*
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		inline
		int set_node_config( gnd::conf::configuration *dest, node_config *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// map file option
			gnd::conf::set_parameter( dest, &src->bmp_map );
			gnd::conf::set_parameter( dest, &src->bmp_map_origin );
			// ros communication option
			gnd::conf::set_parameter( dest, &src->node_name );
			gnd::conf::set_parameter( dest, &src->topic_name_particles );
			gnd::conf::set_parameter( dest, &src->topic_name_particle_weights );
			gnd::conf::set_parameter( dest, &src->topic_name_pointcloud );
			// operating option
			gnd::conf::set_parameter( dest, &src->period );
			gnd::conf::set_parameter( dest, &src->points_select_condition_height_range );
			gnd::conf::set_parameter( dest, &src->points_select_condition_ignore_horizontal_range_lower );
			gnd::conf::set_parameter( dest, &src->points_select_condition_ignore_horizontal_range_upper );
			gnd::conf::set_parameter( dest, &src->points_select_condition_culling_distance );
			// debug option
			gnd::conf::set_parameter( dest, &src->period_cui_status_display );

			return 0;
		}
	}
}
// <--- function definition

#endif // GND_PARTICLE_LOCALIZER_CONFIG_HPP_
