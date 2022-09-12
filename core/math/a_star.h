/*************************************************************************/
/*  a_star.h                                                             */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2022 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2022 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef A_STAR_H
#define A_STAR_H

#include "core/oa_hash_map.h"
#include "core/reference.h"
#include <tuple>

/**
	A* pathfinding algorithm

	@author Juan Linietsky <reduzio@gmail.com>
*/

class AStar : public Reference {
	GDCLASS(AStar, Reference);
	friend class AStar2D;

	struct Octant;

	struct Point {
		Point() :
				neighbours(4u),
				unlinked_neighbours(4u) {}

		int id;
		Vector3 pos;
		real_t weight_scale;
		bool enabled;
		uint32_t parallel_support_layers;

		//the octant this coord is within
		Octant* octant;
		

		OAHashMap<int, Point *> neighbours;
		OAHashMap<int, Point *> unlinked_neighbours;

		// Used for pathfinding.
		Point *prev_point;
		OAHashMap<int, Point*> octant_source_prev_point;
		real_t g_score;
		real_t f_score;
		uint64_t open_pass;
		uint64_t closed_pass;

		
	};

	struct Octant {

		Octant() :
			neighbours(4u),
			unlinked_neighbours(4u) {}

		int id;
		Point* origin;
		Vector3 pos;

		OAHashMap<int, Octant*> neighbours;
		OAHashMap<int, Octant*> unlinked_neighbours;


		//points within the Octant that have an altered weight scale
		PoolVector<int> weighted_points;
		
		real_t weight_scale;

		//what layers are able to use this Octant
		uint32_t parallel_support_layers;

		//points within the octant
		OAHashMap<int, Point*> points;


		// Used for pathfinding.
		Octant* prev_octant;
		real_t g_score;
		real_t f_score;
		uint64_t open_pass;
		uint64_t closed_pass;
		Point* search_point;
		
	};

	struct SortPoints {
		_FORCE_INLINE_ bool operator()(const Point *A, const Point *B) const { // Returns true when the Point A is worse than Point B.
			if (A->f_score > B->f_score) {
				return true;
			} else if (A->f_score < B->f_score) {
				return false;
			} else {
				return A->g_score < B->g_score; // If the f_costs are the same then prioritize the points that are further away from the start.
			}
		}
	};
	struct SortOctants {
		_FORCE_INLINE_ bool operator()(const Octant* A, const Octant* B) const { // Returns true when the Point A is worse than Point B.
			if (A->f_score > B->f_score) {
				return true;
			}
			else if (A->f_score < B->f_score) {
				return false;
			}
			else {
				return A->g_score < B->g_score; // If the f_costs are the same then prioritize the points that are further away from the start.
			}
		}
	};

	struct Segment {
		union {
			struct {
				int32_t u;
				int32_t v;
			};
			uint64_t key;
		};

		enum {
			NONE = 0,
			FORWARD = 1,
			BACKWARD = 2,
			BIDIRECTIONAL = FORWARD | BACKWARD
		};
		unsigned char direction;

		bool operator<(const Segment &p_s) const { return key < p_s.key; }
		Segment() {
			key = 0;
			direction = NONE;
		}
		Segment(int p_from, int p_to) {
			if (p_from < p_to) {
				u = p_from;
				v = p_to;
				direction = FORWARD;
			} else {
				u = p_to;
				v = p_from;
				direction = BACKWARD;
			}
		}
	};

	int last_free_id;
	uint64_t pass;
	uint64_t oct_pass;

	OAHashMap<int, Point *> points;
	OAHashMap<int, Octant *> octants;
	Set<Segment> segments;
	Set<Segment> oct_segments;

	
	

	bool _solve(Point *begin_point, Point *end_point, int relevant_layers, bool use_octants);
	bool _octants_solve(Point* begin_point, Point* end_point, int relevant_layers);
	int _can_path(Point* begin_point, Point* end_point, int relevant_layers, Octant* begin_octant, Octant* end_octant, bool reach_end_point, int prev_octant_id);

protected:
	static void _bind_methods();

	virtual real_t _estimate_cost(int p_from_id, int p_to_id);
	virtual real_t _compute_cost(int p_from_id, int p_to_id);
	virtual real_t _compute_octant_cost(int o_from_id, int o_to_id);
	virtual real_t _estimate_octant_cost(int o_from_id, int o_to_id);

public:
	int get_available_point_id() const;
	


	void add_point(int p_id, const Vector3& p_pos, real_t p_weight_scale = 1, int p_layers = 0);
	void add_octant(int o_id, const PoolVector<int> &pool_points, const Vector3& o_pos, int center_point);
	void remove_octant(int o_id);

	PoolVector<int> debug_octant(int o_id);
	int get_point_octant_id(int p_id);
	PoolVector<int> get_octants();

	void append_as_bulk_array(const PoolVector<real_t> &pool_points , int max_connections, const PoolVector<int> &pool_connections);
	void set_as_bulk_array(const PoolVector<real_t> &pool_points, int max_connections, const PoolVector<int> &pool_connections);

	Vector3 get_point_position(int p_id) const;
	void set_point_position(int p_id, const Vector3 &p_pos);
	real_t get_point_weight_scale(int p_id) const;
	void set_point_weight_scale(int p_id, real_t p_weight_scale);
	void remove_point(int p_id);
	bool has_point(int p_id) const;
	PoolVector<int> get_point_connections(int p_id);
	Array get_points();

	void set_point_disabled(int p_id, bool p_disabled = true);
	bool is_point_disabled(int p_id) const;

	void set_point_layer(int p_id, int layer_index, bool l_enabled = true);
	bool get_point_layer(int p_id,int layer_index) const;
	int get_point_layers_value(int p_id) const;

	void connect_octants(int o_id, int o_with_id, bool bidirectional = true);
	void connect_points(int p_id, int p_with_id, bool bidirectional = true);
	void disconnect_points(int p_id, int p_with_id, bool bidirectional = true);
	bool are_points_connected(int p_id, int p_with_id, bool bidirectional = true) const;
	bool are_octants_connected(int o_id, int o_with_id, bool bidirectional = true) const;

	int get_point_count() const;
	int get_point_capacity() const;
	void reserve_space(int p_num_nodes);
	void clear();

	int get_closest_point(const Vector3 &p_point, bool p_include_disabled = false, int relevant_layers = 0) const;
	Vector3 get_closest_position_in_segment(const Vector3 &p_point) const;

	PoolVector<Vector3> get_point_path(int p_from_id, int p_to_id, int relevant_layers = 0, bool use_octants = false);
	PoolVector<int> get_id_path(int p_from_id, int p_to_id, int relevant_layers = 0, bool use_octants = false);

	PoolVector<uint8_t> get_skipped_connections_of_last_path_array();

	AStar();
	~AStar();
};

class AStar2D : public Reference {
	GDCLASS(AStar2D, Reference);
	AStar astar;

	bool _solve(AStar::Point *begin_point, AStar::Point *end_point);

protected:
	static void _bind_methods();

	virtual real_t _estimate_cost(int p_from_id, int p_to_id);
	virtual real_t _compute_cost(int p_from_id, int p_to_id);

public:
	int get_available_point_id() const;

	void add_point(int p_id, const Vector2 &p_pos, real_t p_weight_scale = 1);
	Vector2 get_point_position(int p_id) const;
	void set_point_position(int p_id, const Vector2 &p_pos);
	real_t get_point_weight_scale(int p_id) const;
	void set_point_weight_scale(int p_id, real_t p_weight_scale);
	void remove_point(int p_id);
	bool has_point(int p_id) const;
	PoolVector<int> get_point_connections(int p_id);
	Array get_points();

	void set_point_disabled(int p_id, bool p_disabled = true);
	bool is_point_disabled(int p_id) const;

	void connect_points(int p_id, int p_with_id, bool p_bidirectional = true);
	void disconnect_points(int p_id, int p_with_id, bool p_bidirectional = true);
	bool are_points_connected(int p_id, int p_with_id, bool p_bidirectional = true) const;

	int get_point_count() const;
	int get_point_capacity() const;
	void reserve_space(int p_num_nodes);
	void clear();

	int get_closest_point(const Vector2 &p_point, bool p_include_disabled = false) const;
	Vector2 get_closest_position_in_segment(const Vector2 &p_point) const;

	PoolVector<Vector2> get_point_path(int p_from_id, int p_to_id);
	PoolVector<int> get_id_path(int p_from_id, int p_to_id);

	AStar2D();
	~AStar2D();
};

#endif // A_STAR_H
