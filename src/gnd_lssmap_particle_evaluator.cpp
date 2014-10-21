/**
 * @file gnd_lssmap_particle_evaluator/src/main.cpp
 * @brief particle evaluator node using laser scan statistics map
 **/

#include "gnd/gnd-multi-platform.h"
#include "gnd/gnd-multi-math.h"
#include "gnd/gnd_lssmap_particle_evaluator.hpp"

#include <stdio.h>
#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include "gnd_particle_localizer/msg_localization_particles2d_stamped.h"
#include "gnd_particle_localizer/msg_localization_particle_weights_stamped.h"
#include "gnd/gnd_rosmsg_reader.hpp"

#include "gnd/gnd-bmp.hpp"
#include "gnd/gnd-lssmap-base.hpp"
#include "gnd/gnd-util.h"
#include "gnd/gnd-random.hpp"

#include <float.h>

typedef std_msgs::Header::_seq_type											sequence_id_t;
typedef sensor_msgs::PointCloud												msg_pointcloud_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_pointcloud_t>				msgreader_pointcloud_t;
typedef gnd_particle_localizer::msg_localization_particles2d_stamped		msg_particles_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_particles_t>				msgreader_particles_t;
typedef gnd_particle_localizer::msg_localization_particle_weights_stamped	msg_particle_weights_t;
typedef msg_particle_weights_t::_weights_type::value_type					particle_weight_t;

int main(int argc, char **argv) {

	gnd::lssmap_particle_evaluator::node_config			node_config;

	{ // ---> start up, read configuration file
		if( argc > 1 ) {
			if( gnd::lssmap_particle_evaluator::fread_node_config( argv[1], &node_config ) < 0 ){
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read config file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( gnd::lssmap_particle_evaluator::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}
	} // <--- start up, read configuration file

	{ // ---> initialize rosÅ@platform
		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
		fprintf(stdout, " node: \"%s\"\n", node_config.node_name.value);
	} // <--- initialize rosÅ@platform


	// ros nodehandle
	ros::NodeHandle			nodehandle;					// node handle
	ros::NodeHandle			nodehandle_private("~");	// private node handle

	ros::Subscriber			subsc_pointcloud;			// point-cloud message subscriber
	msgreader_pointcloud_t	msgreader_pointcloud;		// point-cloud message reader
	msg_pointcloud_t		msg_pointcloud;				// point-cloud message

	ros::Subscriber			subsc_particles;			// particles message subscriber
	msgreader_particles_t	msgreader_particles;		// particles message reader
	msg_particles_t			msg_particles;				// particles message

	ros::Publisher			pub_particle_weights;		// particle weight message publisher
	msg_particle_weights_t	msg_particle_weights;		// particle weight message

	gnd::bmp32_t map;

	{ // ---> initialize
		int phase = 0;
		fprintf(stdout, "---------- initialization ----------\n");

		// ---> show initialize phase task
		if( ros::ok() ) {
			fprintf(stdout, " => initialization task\n");
			fprintf(stdout, "   %d. load map\n", ++phase);
			fprintf(stdout, "   %d. make particles subscriber\n", ++phase);
			fprintf(stdout, "   %d. make point-cloud subscriber\n", ++phase);
			fprintf(stdout, "   %d. make particle weights publisher\n", ++phase);
			fprintf(stdout, "\n");
		} // <--- show initialize phase task

		phase = 0;
		// ---> load map
		if( ros::ok() ) {
			int ret;
			FILE *fp = 0;
			fprintf(stdout, "\n");
			fprintf(stdout, "   => %d. load map\n", ++phase);
			fprintf(stdout, "      ... file path \"%s\"\n", node_config.bmp_map.value);
			fprintf(stdout, "          origin file path \"%s\"\n", node_config.bmp_map_origin.value);

			if( !node_config.bmp_map.value[0] ) {
				fprintf(stderr, "      ... error: bmp_map file path is null\n");
				fprintf(stderr, "          usage: fill \"%s\" item in configuration file\n", node_config.bmp_map.item );
				ros::shutdown();
			}
			else if( (ret = gnd::bmp::read32(node_config.bmp_map.value, &map)) < 0) {
				fprintf(stderr, "      ... error: fail to read bmp_map file \"%s\" %d\n", node_config.bmp_map.value, ret);
				ros::shutdown();
			}
			else if( !(fp = fopen(node_config.bmp_map_origin.value, "r")) ) {
				fprintf(stderr, "      ... error: fail to read bmp_map origin file \"%s\"\n", node_config.bmp_map_origin.value);
				ros::shutdown();
			}
			else {
				double x, y;

				if( fscanf(fp, "%lf %lf\n", &x, &y) < 2) {
					fprintf(stderr, "    ... error: fail to read bmp_map origin file \"%s\"\n", node_config.bmp_map_origin.value);
					ros::shutdown();
				}
				else {
					map.pset_origin(x, y);

					gnd::bmp::write32("load.bmp", &map);

					fprintf(stdout, "    ... ok, output \"load.bmp\" to confirm the load map\n");
				}
				fclose(fp);
			}
		} // <--- load map


		// ---> make particles subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, "   => %d. make particles subscriber\n", ++phase);

			if( !node_config.topic_name_particles.value[0] ) {
				fprintf(stderr, "    ... error: particles topic name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_particles.item );
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_particles.value);

				// allocate buffer
				msgreader_particles.allocate(400);

				// subscribe
				subsc_particles = nodehandle.subscribe(node_config.topic_name_particles.value, 400,
						&msgreader_particles_t::rosmsg_read,
						msgreader_particles.reader_pointer() );

				msg_particles.header.seq = 0;
				msg_particles.header.stamp.fromSec(0.0);
				msg_particles.header.frame_id = "";

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make particles subscriber


		// ---> make point-cloud subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, "   => %d. make point-cloud subscriber\n", ++phase);

			if( !node_config.topic_name_pointcloud.value[0] ) {
				fprintf(stderr, "    ... error: point-cloud topic name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_pointcloud.item );
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pointcloud.value);

				// allocate buffer
				msgreader_pointcloud.allocate(100);

				// subscribe
				subsc_pointcloud = nodehandle.subscribe(node_config.topic_name_pointcloud.value, 100,
						&msgreader_pointcloud_t::rosmsg_read,
						msgreader_pointcloud.reader_pointer() );

				msg_pointcloud.header.seq = 0;
				msg_pointcloud.header.stamp.fromSec(0.0);
				msg_pointcloud.header.frame_id = "";

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make point-cloud subscriber


		// ---> make particle weights publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, "   => %d. make particle weights publisher\n", ++phase);

			if( !node_config.topic_name_particle_weights.value[0] ) {
				fprintf(stderr, "    ... error: laser scan topic name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_particle_weights.item );
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_particle_weights.value );
				pub_particle_weights = nodehandle.advertise<msg_particle_weights_t>(node_config.topic_name_particle_weights.value, 1000 );

				msg_particle_weights.header.stamp.fromSec(0.0);
				msg_particle_weights.header.seq = 0;
				msg_particle_weights.header.frame_id = node_config.node_name.value;

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make particle weights publisher

	} // <--- initialize



	if( ros::ok() ){ // ---> operate
		ros::Rate loop_rate(1000);

		double time_start = 0;
		double time_current = 0;
		double schedule_update_pointcloud = 0;
		double schedule_evaluate_particles = 0;
		double schedule_display = 0;

		particle_weight_t log_likelihood_max = 0;

		struct {
			double height_renge_lower;
			double height_renge_upper;
			double square_hrange_lower;
			double square_hrange_upper;
			double square_range_culling;
		} condition_point_select;

		int nline_display = 0;
		particle_weight_t average_particle_weights_display = 0;
		particle_weight_t min_particle_weight_display = 0;
		particle_weight_t max_particle_weight_display = 0;
		double diff_time_particles_and_pointcloud = 0;
		int cnt_used_points_display = 0;

		sequence_id_t seq_prev_pointcloud = 0;
		sequence_id_t seq_prev_particles = 0;


		fprintf(stdout, "----------operate----------\n");

		{ // ---> initialize time
			time_start = ros::Time::now().toSec();
			time_current = time_start;
			schedule_update_pointcloud = time_current;
			schedule_evaluate_particles = time_current;
			schedule_display = time_current;
		} // <--- initialize time

		{ // ---> initialize point select condition parameter
			condition_point_select.height_renge_lower = node_config.points_select_condition_height_range.value[0];
			condition_point_select.height_renge_upper = node_config.points_select_condition_height_range.value[1];

			if( node_config.points_select_condition_ignore_horizontal_range_lower.value < 0 ) {
				condition_point_select.square_hrange_lower = -1;
			}
			else {
				condition_point_select.square_hrange_lower = gnd_square(node_config.points_select_condition_ignore_horizontal_range_lower.value);
			}

			if( node_config.points_select_condition_ignore_horizontal_range_upper.value < 0 ) {
				condition_point_select.square_hrange_upper = -1;
			}
			else {
				condition_point_select.square_hrange_upper = gnd_square(node_config.points_select_condition_ignore_horizontal_range_upper.value);
			}

			if( node_config.points_select_condition_culling_distance.value < 0) {
				condition_point_select.square_range_culling = -1;
			}
			else {
				condition_point_select.square_range_culling = gnd_square(node_config.points_select_condition_culling_distance.value);
			}

		} // <--- initialize point select condition parameter


		// ---> main loop
		fprintf(stderr, " => %s main loop start\n", node_config.node_name.value);
		while( ros::ok() ) {
			// blocking to avoid the waste of computing resource
			loop_rate.sleep();

			// read topics
			ros::spinOnce();

			// current time
			time_current = ros::Time::now().toSec();

			// ---> update point-cloud
			if( time_current > schedule_update_pointcloud
					&& (msgreader_pointcloud.copy_new( &msg_pointcloud, msg_pointcloud.header.seq ) == 0) ) {
				// next time
				schedule_update_pointcloud = gnd_loop_next(time_current, time_start, node_config.period.value );
			} // <--- update point-cloud

			// ---> evaluate particle weights
			if( time_current > schedule_evaluate_particles
					&& msgreader_particles.is_updated( &msg_pointcloud.header.stamp )
					&& gnd::rosutil::is_sequence_updated(seq_prev_pointcloud, msg_pointcloud.header.seq )) {

				if( msgreader_particles.copy_at_time( &msg_particles, &msg_pointcloud.header.stamp ) != 0 ) {
					// fail to read
				}
				else if( msg_particles.poses.size() == 0) {
					// particles data is invalid (no particle)
				}
				else { // ---> evaluate particles weights
					log_likelihood_max = -FLT_MAX;

					// resize particle weights
					msg_particle_weights.weights.resize( msg_particles.poses.size() );

					// ---> particles scanning loop ( calculate log likelihood  )
					for( unsigned int i = 0; i < msg_particles.poses.size(); i++ ) {
						double cosv, sinv;
						struct {
							double x;
							double y;
						} prev_point;

						// initialize
						msg_particle_weights.weights[i] = 0;

						//
						cosv = cos(msg_particles.poses[i].theta);
						sinv = sin(msg_particles.poses[i].theta);

						// initialize ( large value as it is impossible )
						prev_point.x = 10000;
						prev_point.y = 10000;

						// ---> point-cloud scanning loop ( calculate log likelihood  )
						cnt_used_points_display = 0;
						for( unsigned int j = 0; j < msg_pointcloud.points.size(); j++ ) {
							double x_gl, y_gl;

							{ // ---> point select
								double square_range;
								// check height range
								if( node_config.points_select_condition_height_range.value[0] < node_config.points_select_condition_height_range.value[1]	// if height range is configured
								    && ( msg_pointcloud.points[j].z < condition_point_select.height_renge_lower												// if out of range (lower)
								         || msg_pointcloud.points[j].z > condition_point_select.height_renge_upper ) ) {									// if out of range (upper)
									// ignore
									continue;
								}

								// check horizontal range
								square_range = gnd_square( msg_pointcloud.points[j].x ) + gnd_square( msg_pointcloud.points[j].y );
								if( condition_point_select.square_hrange_lower >= 0 	// if lower horizontal range is configured
										&& square_range < condition_point_select.square_hrange_lower) {
									continue;
								}
								else if( square_range < condition_point_select.square_hrange_upper) {
									continue;
								}

								// check distance from contiguous points
								square_range = gnd_square( msg_pointcloud.points[j].x - prev_point.x ) + gnd_square( msg_pointcloud.points[j].y - prev_point.y );
								if( square_range < condition_point_select.square_range_culling ) {
									continue;
								}

								// update previous selected point
								prev_point.x = msg_pointcloud.points[j].x;
								prev_point.y = msg_pointcloud.points[j].y;
							} // <--- point select

							{ // ---> coordinate transformation
								x_gl = msg_pointcloud.points[j].x * cosv - msg_pointcloud.points[j].y * sinv + msg_particles.poses[i].x;
								y_gl = msg_pointcloud.points[j].x * sinv + msg_pointcloud.points[j].y * cosv + msg_particles.poses[i].y;
							} // <--- coordinate transformation

							{ // ---> refer map to get likelihood and calculate the particle log likelihood
								if( !map.ppointer(x_gl, y_gl) ) {
									// out of map range
									// deal as minimum value except zero
									msg_particle_weights.weights[i] += log( (float) 1.0 );
								}
								else {
									msg_particle_weights.weights[i] += log( (float) map.pvalue(x_gl, y_gl) + 1.0 );
								}
							} // <--- refer map to get likelihood and calculate the particle log likelihood
							cnt_used_points_display++;
						} // <--- point-cloud scanning loop ( calculate log likelihood  )

						// save maximum log likelihood
						log_likelihood_max = msg_particle_weights.weights[i] > log_likelihood_max ? msg_particle_weights.weights[i] : log_likelihood_max;
					} // <--- particles scanning loop ( calculate log likelihood  )


					// <--- particle weights scanning loop ( calculate likelihood ratio as weight )
					average_particle_weights_display = 0;
					min_particle_weight_display = FLT_MAX;
					max_particle_weight_display = 0;
					for( unsigned int i = 0; i < msg_particle_weights.weights.size(); i++ ) {
						msg_particle_weights.weights[i] = exp(msg_particle_weights.weights[i] - log_likelihood_max) + FLT_EPSILON;

						average_particle_weights_display += msg_particle_weights.weights[i];
						max_particle_weight_display = max_particle_weight_display > msg_particle_weights.weights[i] ? max_particle_weight_display : msg_particle_weights.weights[i];
						min_particle_weight_display = min_particle_weight_display < msg_particle_weights.weights[i] ? min_particle_weight_display : msg_particle_weights.weights[i];
					} // <--- particle weights scanning loop ( calculate likelihood ratio as weight )
					average_particle_weights_display /= msg_particle_weights.weights.size();

					// set evaluated particles sequence id
					msg_particle_weights.seq_particles = msg_particles.header.seq;

					// set header
					msg_particle_weights.header.seq++;
					msg_particle_weights.header.stamp = msg_particles.header.stamp;

					// publish
					pub_particle_weights.publish( msg_particle_weights );

					diff_time_particles_and_pointcloud = msg_particles.header.stamp.toSec() - msg_pointcloud.header.stamp.toSec();
				} // <--- evaluate particles weights

				// next time
				schedule_evaluate_particles = gnd_loop_next(time_current, time_start, node_config.period.value );
				// add random value to avoid complete coincidence with resampling period
				schedule_evaluate_particles += gnd::random_uniform() * node_config.period.value / 2.0;


				seq_prev_pointcloud = msg_pointcloud.header.seq;
				seq_prev_particles = msg_particles.header.seq;
			} // <--- evaluate particle weights


			// ---> status display
			if( node_config.period_cui_status_display.value > 0 && time_current > schedule_display ) {
				// clear
				if( nline_display ) {
					::fprintf(stderr, "\x1b[%02dA", nline_display);
					nline_display = 0;
				}

				nline_display++; fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_display++; fprintf(stderr, "\x1b[K   operating time : %6.01lf[sec]\n", time_current - time_start);
				nline_display++; fprintf(stderr, "\x1b[K particle weights : topic name \"%s\" (publish)\n", node_config.topic_name_particle_weights.value );
				nline_display++; fprintf(stderr, "\x1b[K                  : latest seq %d\n", msg_particle_weights.header.seq );
				nline_display++; fprintf(stderr, "\x1b[K                  :    average %6.04lf\n", average_particle_weights_display );
				nline_display++; fprintf(stderr, "\x1b[K                  :  min %6.04lf, max %6.04lf\n", min_particle_weight_display, max_particle_weight_display );
				nline_display++; fprintf(stderr, "\x1b[K                  :  max log likelihood %6.04lf\n", log_likelihood_max );
				nline_display++; fprintf(stderr, "\x1b[K        particles : topic name \"%s\" (subscribe)\n", node_config.topic_name_particles.value );
				nline_display++; fprintf(stderr, "\x1b[K                  : latest seq %d\n", msg_particles.header.seq );
				nline_display++; fprintf(stderr, "\x1b[K                  : size %d [number of particles]\n", msg_particles.poses.size() );
				nline_display++; fprintf(stderr, "\x1b[K      point cloud : topic name \"%s\" (subscribe)\n", node_config.topic_name_pointcloud.value );
				nline_display++; fprintf(stderr, "\x1b[K                  : latest seq %d\n", msg_pointcloud.header.seq );
				nline_display++; fprintf(stderr, "\x1b[K                  : size %d, used %d\n", msg_pointcloud.points.size(), cnt_used_points_display );
				nline_display++; fprintf(stderr, "\x1b[K   data associate : stamp diff %7.04lf [sec] (particles - point-cloud)\n", diff_time_particles_and_pointcloud );

				// next time
				schedule_display = gnd_loop_next( time_current, time_start, node_config.period_cui_status_display.value );
			} // <--- status display

		} // <--- main loop

	} // <--- operate

	{ // ---> finalize
		::fprintf(stderr, "----------finalize----------\n");

		::fprintf(stderr, "   ... end\n");
	} // <--- finalize

	return 0;
}
