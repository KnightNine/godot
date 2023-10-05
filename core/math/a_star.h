/**************************************************************************/
/*  a_star.h                                                              */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#ifndef A_STAR_H
#define A_STAR_H

#include "core/object/gdvirtual.gen.inc"
#include "core/object/ref_counted.h"
#include "core/templates/oa_hash_map.h"

/**
	A* pathfinding algorithm.
*/

class AStar3D : public RefCounted {
	GDCLASS(AStar3D, RefCounted);
	friend class AStar2D;

	struct Octant;

	struct Point {
		Point() {}

		int64_t id = 0;
		Vector3 pos;
		real_t weight_scale = 0;
		bool enabled = false;

		uint32_t nav_layers = 0;

		//the octant this coord is within
		Octant* octant;

		OAHashMap<int64_t, Point *> neighbors = 4u;
		OAHashMap<int64_t, Point *> unlinked_neighbours = 4u;
		OAHashMap<int64_t, Point*> octant_source_prev_point = 4u;

		// Used for pathfinding.
		Point *prev_point = nullptr;
		real_t g_score = 0;
		real_t f_score = 0;
		uint64_t open_pass = 0;
		uint64_t closed_pass = 0;

		//used for getting closest_point_of_last_pathing_call
		real_t abs_g_score = 0;
		real_t abs_f_score = 0;
	};

	struct Octant {

		Octant() {}

		int64_t id = 0;
		Point* origin = nullptr;
		Vector3 pos;

		OAHashMap<int64_t, Octant*> neighbours = 4u;
		OAHashMap<int64_t, Octant*> unlinked_neighbours = 4u;
		//points within the octant
		OAHashMap<int64_t, Point*> points = 4u;

		//points within the Octant that have an altered weight scale
		Vector<int64_t> weighted_points;

		real_t weight_scale = 0;

		//what layers are able to use this Octant
		uint32_t nav_layers = 0;

		


		// Used for pathfinding.
		Vector<Octant*> prev_octants;
		Octant* prev_octant = nullptr;
		real_t g_score = 0;
		real_t f_score = 0;
		uint64_t open_pass = 0;
		uint64_t closed_pass = 0;
		Point* search_point = nullptr;


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
		Pair<int64_t, int64_t> key;

		enum {
			NONE = 0,
			FORWARD = 1,
			BACKWARD = 2,
			BIDIRECTIONAL = FORWARD | BACKWARD
		};
		unsigned char direction = NONE;

		static uint32_t hash(const Segment &p_seg) {
			return PairHash<int64_t, int64_t>().hash(p_seg.key);
		}
		bool operator==(const Segment &p_s) const { return key == p_s.key; }

		Segment() {}
		Segment(int64_t p_from, int64_t p_to) {
			if (p_from < p_to) {
				key.first = p_from;
				key.second = p_to;
				direction = FORWARD;
			} else {
				key.first = p_to;
				key.second = p_from;
				direction = BACKWARD;
			}
		}
	};

	int64_t last_free_id = 0;
	uint64_t pass = 1;
	uint64_t oct_pass = 1;



	OAHashMap<int64_t, Point *> points;
	OAHashMap<int64_t, Octant*> octants;
	HashSet<Segment, Segment> segments;
	HashSet<Segment, Segment> oct_segments;

	Vector<int64_t> id_path_of_last_pathing_call;
	Vector<Vector3> point_path_of_last_pathing_call;
	Point* closest_point_of_last_pathing_call;

	StringName straight_line_function;
	ObjectID function_source_id;

	bool _solve(Point *begin_point, Point *end_point, int32_t relevant_layers, bool use_octants);
	bool _octants_solve(Point* begin_point, Point* end_point, int32_t relevant_layers);
	int _can_path(Point* begin_point, Point* end_point, int32_t relevant_layers, Octant* begin_octant, Octant* end_octant, bool reach_end_point, int64_t prev_octant_id, Point* absolute_begin_point, Point* absolute_end_point);

protected:
	static void _bind_methods();

	virtual real_t _estimate_cost(int64_t p_from_id, int64_t p_to_id);
	virtual real_t _compute_cost(int64_t p_from_id, int64_t p_to_id);
	virtual real_t _compute_octant_cost(int64_t o_from_id, int64_t o_to_id);
	virtual real_t _estimate_octant_cost(int64_t o_from_id, int64_t o_to_id);

	GDVIRTUAL2RC(real_t, _estimate_cost, int64_t, int64_t)
	GDVIRTUAL2RC(real_t, _compute_cost, int64_t, int64_t)
	GDVIRTUAL2RC(real_t, _estimate_octant_cost, int64_t, int64_t)
	GDVIRTUAL2RC(real_t, _compute_octant_cost, int64_t, int64_t)


public:
	int64_t get_available_point_id() const;

	void add_point(int64_t p_id, const Vector3 &p_pos, real_t p_weight_scale = 1, int32_t p_layers = 0);
	void add_octant(int64_t o_id, const PackedInt64Array &pool_points, const Vector3 &o_pos, int64_t center_point);
	void remove_octant(int64_t o_id);

	PackedInt64Array debug_octant(int64_t o_id);
	int64_t get_point_octant_id(int64_t p_id);
	PackedInt64Array get_octant_ids();

	void append_as_bulk_array(const PackedFloat64Array&pool_points, int64_t max_connections, const PackedInt64Array& pool_connections);
	void set_as_bulk_array(const PackedFloat64Array& pool_points, int64_t max_connections, const PackedInt64Array& pool_connections);

	Vector3 get_point_position(int64_t p_id) const;
	void set_point_position(int64_t p_id, const Vector3 &p_pos);
	real_t get_point_weight_scale(int64_t p_id) const;
	void set_point_weight_scale(int64_t p_id, real_t p_weight_scale);
	void remove_point(int64_t p_id);
	bool has_point(int64_t p_id) const;
	Vector<int64_t> get_point_connections(int64_t p_id);
	PackedInt64Array get_point_ids();

	void set_point_disabled(int64_t p_id, bool p_disabled = true);
	bool is_point_disabled(int64_t p_id) const;

	void set_point_layer(int64_t p_id, int64_t layer_index, bool l_enabled = true);
	void set_point_layers_value(int64_t p_id, int64_t p_layers);
	bool get_point_layer(int64_t p_id, int64_t layer_index) const;
	int32_t get_point_layers_value(int64_t p_id) const;

	bool set_straight_line_function(Object* p_obj, const StringName& draw_straight_line_f_name);
	PackedInt64Array _get_straight_line(int64_t from_p_id, int64_t to_p_id);

	void connect_octants(int64_t o_id, int64_t o_with_id, bool bidirectional = true);
	bool are_octants_connected(int64_t o_id, int64_t o_with_id, bool bidirectional = true) const;

	void connect_points(int64_t p_id, int64_t p_with_id, bool bidirectional = true);
	void disconnect_points(int64_t p_id, int64_t p_with_id, bool bidirectional = true);
	bool are_points_connected(int64_t p_id, int64_t p_with_id, bool bidirectional = true) const;

	int64_t get_point_count() const;
	int64_t get_point_capacity() const;
	void reserve_space(int64_t p_num_nodes);
	void clear();

	int64_t get_closest_point(const Vector3 &p_point, bool p_include_disabled = false, int32_t relevant_layers = 0) const;
	Vector3 get_closest_position_in_segment(const Vector3 &p_point) const;

	Vector<int64_t> get_proximity_id_path_of_last_pathing_call();
	Vector<Vector3> get_proximity_point_path_of_last_pathing_call();

	Vector<Vector3> get_point_path(int64_t p_from_id, int64_t p_to_id, int32_t relevant_layers = 0, bool use_octants = false);
	Vector<int64_t> get_id_path(int64_t p_from_id, int64_t p_to_id, int32_t relevant_layers = 0, bool use_octants = false);

	AStar3D() {}
	~AStar3D();
};

class AStar2D : public RefCounted {
	GDCLASS(AStar2D, RefCounted);
	AStar3D astar;

	bool _solve(AStar3D::Point *begin_point, AStar3D::Point *end_point);

protected:
	static void _bind_methods();

	virtual real_t _estimate_cost(int64_t p_from_id, int64_t p_to_id);
	virtual real_t _compute_cost(int64_t p_from_id, int64_t p_to_id);

	GDVIRTUAL2RC(real_t, _estimate_cost, int64_t, int64_t)
	GDVIRTUAL2RC(real_t, _compute_cost, int64_t, int64_t)

public:
	int64_t get_available_point_id() const;

	void add_point(int64_t p_id, const Vector2 &p_pos, real_t p_weight_scale = 1);
	Vector2 get_point_position(int64_t p_id) const;
	void set_point_position(int64_t p_id, const Vector2 &p_pos);
	real_t get_point_weight_scale(int64_t p_id) const;
	void set_point_weight_scale(int64_t p_id, real_t p_weight_scale);
	void remove_point(int64_t p_id);
	bool has_point(int64_t p_id) const;
	Vector<int64_t> get_point_connections(int64_t p_id);
	PackedInt64Array get_point_ids();

	void set_point_disabled(int64_t p_id, bool p_disabled = true);
	bool is_point_disabled(int64_t p_id) const;

	void connect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional = true);
	void disconnect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional = true);
	bool are_points_connected(int64_t p_id, int64_t p_with_id, bool p_bidirectional = true) const;

	int64_t get_point_count() const;
	int64_t get_point_capacity() const;
	void reserve_space(int64_t p_num_nodes);
	void clear();

	int64_t get_closest_point(const Vector2 &p_point, bool p_include_disabled = false) const;
	Vector2 get_closest_position_in_segment(const Vector2 &p_point) const;

	Vector<Vector2> get_point_path(int64_t p_from_id, int64_t p_to_id);
	Vector<int64_t> get_id_path(int64_t p_from_id, int64_t p_to_id);

	AStar2D() {}
	~AStar2D() {}
};

#endif // A_STAR_H
