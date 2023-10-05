/**************************************************************************/
/*  a_star.cpp                                                            */
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

#include "a_star.h"

#include "core/math/geometry_3d.h"
#include "core/object/script_language.h"


void AStar3D::set_debug_mode(bool is_active) {
	debug_mode_active = is_active;
}

int64_t AStar3D::get_available_point_id() const {
	if (points.has(last_free_id)) {
		int64_t cur_new_id = last_free_id + 1;
		while (points.has(cur_new_id)) {
			cur_new_id++;
		}
		const_cast<int64_t &>(last_free_id) = cur_new_id;
	}

	return last_free_id;
}

void AStar3D::add_point(int64_t p_id, const Vector3 &p_pos, real_t p_weight_scale, int32_t p_layers) {
	ERR_FAIL_COND_MSG(p_id < 0, vformat("Can't add a point with negative id: %d.", p_id));
	ERR_FAIL_COND_MSG(p_weight_scale < 0.0, vformat("Can't add a point with weight scale less than 0.0: %f.", p_weight_scale));
	ERR_FAIL_INDEX_MSG(p_layers, ((1 << 31) - 1), vformat("Can't add a point with layers value less than 0 or more than 2^31 - 1: %d.", p_layers));

	Point *found_pt;
	bool p_exists = points.lookup(p_id, found_pt);

	if (!p_exists) {
		Point *pt = memnew(Point);
		pt->id = p_id;
		pt->pos = p_pos;
		pt->weight_scale = p_weight_scale;
		pt->nav_layers = p_layers;
		pt->prev_point = nullptr;
		pt->open_pass = 0;
		pt->closed_pass = 0;
		pt->enabled = true;
		points.set(p_id, pt);
	} else {
		found_pt->pos = p_pos;
		set_point_weight_scale(found_pt->id, p_weight_scale);
		set_point_layers_value(found_pt->id, p_layers);
	}
}

void AStar3D::add_octant(int64_t o_id, const PackedInt64Array& pool_points, const Vector3& o_pos, int64_t center_point) {
	ERR_FAIL_COND_MSG(o_id < 0, vformat("Can't add an octant with negative id: %d.", o_id));
	Octant* found_oc;
	bool o_exists = octants.lookup(o_id, found_oc);

	uint32_t nav_layers = 0;
	const int64_t* r = pool_points.ptr();
	int size = pool_points.size();
	ERR_FAIL_COND_MSG(size <= 0, vformat("Can't add an octant with zero pool_points: %d.", o_id));







	//if placed overlapping with an existing octant's points, remove this octant
	bool invalid = false;
	int invalid_type = 0;
	int overlapping_p_id = 0;


	if (!o_exists) {
		Octant* oc = memnew(Octant);
		oc->id = o_id;
		oc->pos = o_pos;
		oc->weight_scale = 1;
		oc->prev_octant = nullptr;
		oc->open_pass = 0;
		oc->closed_pass = 0;
		oc->search_point = nullptr;
		oc->origin = nullptr;
		
		

		for (int i = 0; i < size; i++) {

			int64_t p_id = r[i];
			Point* p;
			bool p_exists = points.lookup(p_id, p);




			if (p_exists) {
				if (p_id == center_point) {
					oc->origin = p;
				}
				//this will add layers that are supported within the octant
				nav_layers = p->nav_layers | nav_layers;


				//cannot overlap with other octant points
				if (p->octant == nullptr) {
					p->octant = oc;

					oc->points.set(p_id, p);




					if (p->weight_scale != real_t(1)) {

						oc->weighted_points.push_back(p_id);
						oc->weight_scale += p->weight_scale - 1 / size;
					}
				}
				else {
					invalid = true;
					overlapping_p_id = p_id;

					break;
				}


			}
			else {
				invalid = true;
				invalid_type = 1;
				break;
			}

		}


		if (oc->origin == nullptr && !invalid) {
			invalid = true;
			invalid_type = 2;

		}


		oc->nav_layers = nav_layers;

		octants.set(o_id, oc);
	}
	else {

		found_oc->pos = o_pos;
		found_oc->origin = nullptr;
		//clear old points
		for (OAHashMap<int64_t, Point*>::Iterator it = found_oc->points.iter(); it.valid; it = found_oc->points.next_iter(it)) {
			Point* p = *it.value;
			p->octant = nullptr;
		}

		found_oc->points.clear();
		found_oc->weighted_points.clear();
		found_oc->weight_scale = 1;

		

		for (int i = 0; i < size; i++) {

			int64_t p_id = r[i];
			Point* p;
			bool p_exists = points.lookup(p_id, p);
			if (p_exists) {
				if (p_id == center_point) {
					found_oc->origin = p;
				}
				//this will add layers that are supported within the octant
				nav_layers = p->nav_layers | nav_layers;

				//cannot overlap with other octant points
				if (p->octant == nullptr) {
					p->octant = found_oc;

					found_oc->points.set(p_id, p);


					if (p->weight_scale != real_t(1)) {

						found_oc->weighted_points.push_back(p_id);
						found_oc->weight_scale += p->weight_scale - 1 / size;
					}
				}
				else {
					invalid = true;
					overlapping_p_id = p_id;
					break;
				}

			}
			else {
				invalid = true;
				invalid_type = 1;
				break;
			}

		}

		if (found_oc->origin == nullptr && !invalid) {
			invalid = true;
			invalid_type = 2;

		}


		// only enabled when containing no disabled points or weighted points

		found_oc->nav_layers = nav_layers;
	}

	if (invalid) {

		remove_octant(o_id);

		//usure how i would go about printing an error without cancelling the function
		if (invalid_type == 2) {
			ERR_FAIL_COND_MSG(invalid, vformat("octant placement of id %d does not contain the defined center_point and is therefore invalid and has been removed", o_id));
		}

		if (invalid_type == 1) {
			ERR_FAIL_COND_MSG(invalid, vformat("octant placement of id %d contains points which do not exist and is therefore invalid and has been removed", o_id));
		}
		ERR_FAIL_COND_MSG(invalid, vformat("octant placement of id %d overlaps with another octant at point %d and is therefore invalid and has been removed", o_id, overlapping_p_id));
	}

}


//returns int array 
PackedInt64Array AStar3D::debug_octant(int64_t o_id) {
	Octant* o;
	bool o_exists = octants.lookup(o_id, o);
	ERR_FAIL_COND_V_MSG(!o_exists, PackedInt64Array(), vformat("Can't debug octant. Octant with id: %d doesn't exist.", o_id));

	//debug_data = [octant_layers,points_list]

	PackedInt64Array debug_data;


	int layers = o->nav_layers;
	debug_data.append(layers);



	if (o->weighted_points.size() > 0) {
		debug_data.append(0); // 0 if weighted_points
		const int64_t* r = o->weighted_points.ptr();
		for (int i = 0; i < o->weighted_points.size(); i++) {
			debug_data.append(r[i]);
		}
	}


	return debug_data;

}

int64_t AStar3D::get_point_octant_id(int64_t p_id) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, -1, vformat("Can't get if point has octant_id. Point with id: %d doesn't exist.", p_id));

	Octant* o = p->octant;
	if (o != nullptr) {
		return o->id;
	}
	return -1;

}


PackedInt64Array AStar3D::get_octant_ids() {
	PackedInt64Array o_ids;
	for (OAHashMap<int64_t, Octant*>::Iterator it = octants.iter(); it.valid; it = octants.next_iter(it)) {
		int64_t o_id = *it.key;
		o_ids.append(o_id);
	}

	return o_ids;
}

void AStar3D::remove_octant(int64_t o_id) {
	Octant* o;
	bool o_exists = octants.lookup(o_id, o);
	ERR_FAIL_COND_MSG(!o_exists, vformat("Can't remove octant. Octant with id: %d doesn't exist.", o_id));


	for (OAHashMap<int64_t, Point*>::Iterator it = o->points.iter(); it.valid; it = o->points.next_iter(it)) {
		Point* p = *it.value;
		p->octant = nullptr;


	}

	for (OAHashMap<int64_t, Octant*>::Iterator it = o->neighbours.iter(); it.valid; it = o->neighbours.next_iter(it)) {
		Segment s(o_id, (*it.key));
		oct_segments.erase(s);

		(*it.value)->neighbours.remove(o->id);
		(*it.value)->unlinked_neighbours.remove(o->id);
	}

	for (OAHashMap<int64_t, Octant*>::Iterator it = o->unlinked_neighbours.iter(); it.valid; it = o->unlinked_neighbours.next_iter(it)) {
		Segment s(o_id, (*it.key));
		oct_segments.erase(s);

		(*it.value)->neighbours.remove(o->id);
		(*it.value)->unlinked_neighbours.remove(o->id);
	}


	memdelete(o);
	octants.remove(o_id);


}

void AStar3D::append_as_bulk_array(const PackedFloat64Array& pool_points, int64_t max_connections, const PackedInt64Array& pool_connections)
{






	int size = pool_points.size();

	ERR_FAIL_COND_MSG(size % 6 > 0, vformat("pool_points size lacks data for each point"));

	const double* r = pool_points.ptr();
	
	for (int i = 0; i < size / 6; i++) {
		
		int64_t p_id = r[i * 6 + 0];
		real_t x = r[i * 6 + 1];
		real_t y = r[i * 6 + 2];
		real_t z = r[i * 6 + 3];
		real_t p_weight_scale = r[i * 6 + 4];
		int32_t p_layers = r[i * 6 + 5];
		Vector3 p_pos = Vector3(x, y, z);
		add_point(p_id, p_pos, p_weight_scale, p_layers);

	}

	const int64_t* r1 = pool_connections.ptr();
	size = pool_connections.size();
	int i_mult = max_connections + 1;
	ERR_FAIL_COND_MSG(size % i_mult > 0, vformat("pool_connections size lacks data for each point"));

	for (int i = 0; i < size / i_mult; i++) {

		int64_t p_id = r1[i * i_mult + 0];
		for (int j = 1; j < i_mult; j++) {

			int64_t p_with_id = r1[i * i_mult + j];
			if (p_with_id >= 0) {
				connect_points(p_id, p_with_id);
			}

		}




	}

}

// this may be faster than doing it in gdscript
void AStar3D::set_as_bulk_array(const PackedFloat64Array& pool_points, int64_t max_connections, const PackedInt64Array& pool_connections)
{

	//wipe all points
	clear();

	int size = pool_points.size();
	ERR_FAIL_COND_MSG(size % 6 > 0, vformat("pool_points size lacks data for each point"));
	const double* r = pool_points.ptr();
	
	for (int i = 0; i < size / 6; i++) {

		int64_t p_id = r[i * 6 + 0];
		real_t x = r[i * 6 + 1];
		real_t y = r[i * 6 + 2];
		real_t z = r[i * 6 + 3];
		real_t p_weight_scale = r[i * 6 + 4];
		int64_t p_layers = r[i * 6 + 5];

		ERR_FAIL_COND_MSG(p_id < 0, vformat("Can't add a point with negative id: %d.", p_id));
		ERR_FAIL_COND_MSG(p_weight_scale < 0, vformat("Can't add a point with weight scale less than 0.0: %f.", p_weight_scale));
		ERR_FAIL_INDEX_MSG(p_layers, ((1 << 31) - 1), vformat("Can't add a point with layers value less than 0 or more than 2^31 - 1: %d.", p_layers));

		Vector3 p_pos = Vector3(x, y, z);

		Point* pt = memnew(Point);
		pt->id = p_id;
		pt->pos = p_pos;
		pt->weight_scale = p_weight_scale;
		pt->nav_layers = int32_t(p_layers);
		pt->octant = nullptr;
		pt->prev_point = nullptr;
		pt->open_pass = 0;
		pt->closed_pass = 0;
		pt->enabled = true;
		points.set(p_id, pt);



	}

	const int64_t* r1 = pool_connections.ptr();
	size = pool_connections.size();
	int i_mult = max_connections + 1;
	ERR_FAIL_COND_MSG(size % i_mult > 0, vformat("pool_connections size lacks data for each point"));

	for (int i = 0; i < size / i_mult; i++) {

		int64_t p_id = r1[i * i_mult + 0];
		for (int j = 1; j < i_mult; j++) {

			int64_t p_with_id = r1[i * i_mult + j];
			if (p_with_id >= 0) {
				connect_points(p_id, p_with_id);
			}

		}




	}
}

Vector3 AStar3D::get_point_position(int64_t p_id) const {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, Vector3(), vformat("Can't get point's position. Point with id: %d doesn't exist.", p_id));

	return p->pos;
}

void AStar3D::set_point_position(int64_t p_id, const Vector3 &p_pos) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point's position. Point with id: %d doesn't exist.", p_id));

	p->pos = p_pos;
}

real_t AStar3D::get_point_weight_scale(int64_t p_id) const {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, 0, vformat("Can't get point's weight scale. Point with id: %d doesn't exist.", p_id));

	return p->weight_scale;
}

void AStar3D::set_point_weight_scale(int64_t p_id, real_t p_weight_scale) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point's weight scale. Point with id: %d doesn't exist.", p_id));
	ERR_FAIL_COND_MSG(p_weight_scale < 0.0, vformat("Can't set point's weight scale less than 0.0: %f.", p_weight_scale));

	real_t original_ws = p->weight_scale;

	p->weight_scale = p_weight_scale;

	// if point is part of an octant, adjust the octant's weight scale 
	if (p->octant != nullptr) {
		Octant* o = p->octant;

		//weight scale of octant is the average of the weight scales of all points it contains.
		int octant_points_size = o->points.get_num_elements();

		//remove point old weight scale from octant
		o->weight_scale -= (original_ws - 1) / octant_points_size;
		//remove from weighted points
		o->weighted_points.erase(p_id);
		

		if (p->weight_scale != real_t(1)) {
			//add to weighted points
			o->weighted_points.push_back(p_id);
			//add point new weight scale to octant
			o->weight_scale += (p_weight_scale - 1) / octant_points_size;
		}
		else {
			//point is no longer weighted
			


			//reset octant weight scale if all weighted points removed, this is to negate any floating point inaccuracies which may accumulate
			if (o->weighted_points.size() == 0) {
				o->weight_scale = 1;
			}



		}

	}
}

void AStar3D::remove_point(int64_t p_id) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't remove point. Point with id: %d doesn't exist.", p_id));

	for (OAHashMap<int64_t, Point *>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
		Segment s(p_id, (*it.key));
		segments.erase(s);

		(*it.value)->neighbors.remove(p->id);
		(*it.value)->unlinked_neighbours.remove(p->id);
	}

	for (OAHashMap<int64_t, Point *>::Iterator it = p->unlinked_neighbours.iter(); it.valid; it = p->unlinked_neighbours.next_iter(it)) {
		Segment s(p_id, (*it.key));
		segments.erase(s);

		(*it.value)->neighbors.remove(p->id);
		(*it.value)->unlinked_neighbours.remove(p->id);
	}

	//remove the octant if any
	if (p->octant != nullptr) {
		remove_octant(p->octant->id);

	}

	memdelete(p);
	points.remove(p_id);
	last_free_id = p_id;
}
void AStar3D::connect_octants(int64_t o_id, int64_t o_with_id, bool bidirectional) {
	ERR_FAIL_COND_MSG(o_id == o_with_id, vformat("Can't connect octant with id: %d to itself.", o_id));

	Octant* a;
	bool from_exists = octants.lookup(o_id, a);
	ERR_FAIL_COND_MSG(!from_exists, vformat("Can't connect octants. Octant with id: %d doesn't exist.", o_id));

	Octant* b;
	bool to_exists = octants.lookup(o_with_id, b);
	ERR_FAIL_COND_MSG(!to_exists, vformat("Can't connect octants. Octant with id: %d doesn't exist.", o_with_id));

	a->neighbours.set(b->id, b);

	if (bidirectional) {
		b->neighbours.set(a->id, a);
	}
	else {
		b->unlinked_neighbours.set(a->id, a);
	}

	Segment s(o_id, o_with_id);
	if (bidirectional) {
		s.direction = Segment::BIDIRECTIONAL;
	}

	HashSet<Segment, Segment>::Iterator element = oct_segments.find(s);
	if (element) {
		s.direction |= element->direction;
		if (s.direction == Segment::BIDIRECTIONAL) {
			// Both are neighbours of each other now
			a->unlinked_neighbours.remove(b->id);
			b->unlinked_neighbours.remove(a->id);
		}
		oct_segments.remove(element);
	}

	oct_segments.insert(s);
}

void AStar3D::connect_points(int64_t p_id, int64_t p_with_id, bool bidirectional) {
	ERR_FAIL_COND_MSG(p_id == p_with_id, vformat("Can't connect point with id: %d to itself.", p_id));

	Point *a;
	bool from_exists = points.lookup(p_id, a);
	ERR_FAIL_COND_MSG(!from_exists, vformat("Can't connect points. Point with id: %d doesn't exist.", p_id));

	Point *b;
	bool to_exists = points.lookup(p_with_id, b);
	ERR_FAIL_COND_MSG(!to_exists, vformat("Can't connect points. Point with id: %d doesn't exist.", p_with_id));

	a->neighbors.set(b->id, b);

	if (bidirectional) {
		b->neighbors.set(a->id, a);
	} else {
		b->unlinked_neighbours.set(a->id, a);
	}

	Segment s(p_id, p_with_id);
	if (bidirectional) {
		s.direction = Segment::BIDIRECTIONAL;
	}

	HashSet<Segment, Segment>::Iterator element = segments.find(s);
	if (element) {
		s.direction |= element->direction;
		if (s.direction == Segment::BIDIRECTIONAL) {
			// Both are neighbors of each other now
			a->unlinked_neighbours.remove(b->id);
			b->unlinked_neighbours.remove(a->id);
		}
		segments.remove(element);
	}

	segments.insert(s);
}

void AStar3D::disconnect_points(int64_t p_id, int64_t p_with_id, bool bidirectional) {
	Point *a;
	bool a_exists = points.lookup(p_id, a);
	ERR_FAIL_COND_MSG(!a_exists, vformat("Can't disconnect points. Point with id: %d doesn't exist.", p_id));

	Point *b;
	bool b_exists = points.lookup(p_with_id, b);
	ERR_FAIL_COND_MSG(!b_exists, vformat("Can't disconnect points. Point with id: %d doesn't exist.", p_with_id));

	Segment s(p_id, p_with_id);
	int remove_direction = bidirectional ? (int)Segment::BIDIRECTIONAL : (int)s.direction;

	HashSet<Segment, Segment>::Iterator element = segments.find(s);
	if (element) {
		// s is the new segment
		// Erase the directions to be removed
		s.direction = (element->direction & ~remove_direction);

		a->neighbors.remove(b->id);
		if (bidirectional) {
			b->neighbors.remove(a->id);
			if (element->direction != Segment::BIDIRECTIONAL) {
				a->unlinked_neighbours.remove(b->id);
				b->unlinked_neighbours.remove(a->id);
			}
		} else {
			if (s.direction == Segment::NONE) {
				b->unlinked_neighbours.remove(a->id);
			} else {
				a->unlinked_neighbours.set(b->id, b);
			}
		}

		segments.remove(element);
		if (s.direction != Segment::NONE) {
			segments.insert(s);
		}
	}
}

bool AStar3D::has_point(int64_t p_id) const {
	return points.has(p_id);
}

PackedInt64Array AStar3D::get_point_ids() {
	PackedInt64Array point_list;

	for (OAHashMap<int64_t, Point *>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		point_list.push_back(*(it.key));
	}

	return point_list;
}

Vector<int64_t> AStar3D::get_point_connections(int64_t p_id) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, Vector<int64_t>(), vformat("Can't get point's connections. Point with id: %d doesn't exist.", p_id));

	Vector<int64_t> point_list;

	for (OAHashMap<int64_t, Point *>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
		point_list.push_back((*it.key));
	}

	return point_list;
}

bool AStar3D::are_points_connected(int64_t p_id, int64_t p_with_id, bool bidirectional) const {
	Segment s(p_id, p_with_id);
	const HashSet<Segment, Segment>::Iterator element = segments.find(s);

	return element &&
			(bidirectional || (element->direction & s.direction) == s.direction);
}

//note: this crashes if an id is -1
bool AStar3D::are_octants_connected(int64_t o_id, int64_t o_with_id, bool bidirectional) const {
	Segment s(o_id, o_with_id);
	const HashSet<Segment, Segment>::Iterator element = oct_segments.find(s);

	return element &&
		(bidirectional || (element->direction & s.direction) == s.direction);
}

void AStar3D::clear() {
	last_free_id = 0;
	for (OAHashMap<int64_t, Octant*>::Iterator it = octants.iter(); it.valid; it = octants.next_iter(it)) {
		memdelete(*(it.value));
	}
	for (OAHashMap<int64_t, Point *>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		memdelete(*(it.value));
	}

	segments.clear();
	oct_segments.clear();
	octants.clear();
	points.clear();
}

int64_t AStar3D::get_point_count() const {
	return points.get_num_elements();
}

int64_t AStar3D::get_point_capacity() const {
	return points.get_capacity();
}

void AStar3D::reserve_space(int64_t p_num_nodes) {
	ERR_FAIL_COND_MSG(p_num_nodes <= 0, vformat("New capacity must be greater than 0, new was: %d.", p_num_nodes));
	ERR_FAIL_COND_MSG((uint32_t)p_num_nodes < points.get_capacity(), vformat("New capacity must be greater than current capacity: %d, new was: %d.", points.get_capacity(), p_num_nodes));
	points.reserve(p_num_nodes);
}

int64_t AStar3D::get_closest_point(const Vector3 &p_point, bool p_include_disabled, int32_t relevant_layers) const {
	int64_t closest_id = -1;
	real_t closest_dist = 1e20;

	for (OAHashMap<int64_t, Point *>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		//make sure parallel layers are supported
		// or if *relevant_layers is 0 then use all points
		bool supported = relevant_layers == 0 || (relevant_layers & (*it.value)->nav_layers) > 0;

		if (!p_include_disabled && !(*it.value)->enabled || !supported) {
			continue; // Disabled points should not be considered.
		}

		// Keep the closest point's ID, and in case of multiple closest IDs,
		// the smallest one (makes it deterministic).
		real_t d = p_point.distance_squared_to((*it.value)->pos);
		int64_t id = *(it.key);
		if (d <= closest_dist) {
			if (d == closest_dist && id > closest_id) { // Keep lowest ID.
				continue;
			}
			closest_dist = d;
			closest_id = id;
		}
	}

	return closest_id;
}

Vector3 AStar3D::get_closest_position_in_segment(const Vector3 &p_point) const {
	real_t closest_dist = 1e20;
	Vector3 closest_point;

	for (const Segment &E : segments) {
		Point *from_point = nullptr, *to_point = nullptr;
		points.lookup(E.key.first, from_point);
		points.lookup(E.key.second, to_point);

		if (!(from_point->enabled && to_point->enabled)) {
			continue;
		}

		Vector3 segment[2] = {
			from_point->pos,
			to_point->pos,
		};

		Vector3 p = Geometry3D::get_closest_point_to_segment(p_point, segment);
		real_t d = p_point.distance_squared_to(p);
		if (d < closest_dist) {
			closest_point = p;
			closest_dist = d;
		}
	}

	return closest_point;
}

bool AStar3D::_octants_solve(Point* begin_point, Point* end_point, int32_t relevant_layers) {
	oct_pass++;



	//make sure parallel layers are supported
	// or if *relevant_layers is 0 then use all points
	bool supported = relevant_layers == 0 || (relevant_layers & end_point->nav_layers) > 0;
	if (!end_point->enabled || !supported) {
		return false;
	}

	bool found_route = false;

	LocalVector<Octant*> oct_open_list;
	SortArray<Octant*, SortOctants> oct_sorter;



	Octant* begin_octant = begin_point->octant;
	begin_octant->search_point = begin_point;

	Octant* end_octant = end_point->octant;




	_debug_print(vformat("	pathing from: %d to: %d", begin_octant->id, end_octant->id));

	begin_octant->g_score = 0;
	begin_octant->f_score = _estimate_octant_cost(begin_octant->id, end_octant->id);
	begin_octant->prev_octant = nullptr;

	//check if empty before resizing in order to not throw an error
	begin_octant->prev_octants.clear();


	oct_open_list.push_back(begin_octant);



	while (!oct_open_list.is_empty()) {
		Octant* o = oct_open_list[0]; // The currently processed octant

		//_debug_print(vformat("running through octant of index %d.", o->id));

		oct_sorter.pop_heap(0, oct_open_list.size(), oct_open_list.ptr()); // Remove the current octant from the open list
		oct_open_list.remove_at(oct_open_list.size() - 1);







		int size = o->prev_octants.size();

		if (size > 0) {
			//try pathing from the previous octant to this octant (after sorting the neighbors, in order to minimize the number of _can_path() checks needed)
			int connection = -1;


			//try each previous octant for a valid connection:
			//? how do i use ptr() for this?
			//const Octant * r = o->prev_octants.ptr();
			int i = 0;
			while (connection == -1 && i < size) {

				Octant* prev_octant = o->prev_octants[i];//r[i];
				_debug_print(vformat("%d _can_path octant %d", o->id, prev_octant->id));
				//id of the previous octant before the previous octant
				int64_t ppo_id = -1;
				if (prev_octant->prev_octant != nullptr) {
					ppo_id = prev_octant->prev_octant->id;
				}


				if (o == end_octant) {
					//if this is the end_octant, reaching the end_point from the last octant is required in order for the octant connection to be valid
					connection = _can_path(prev_octant->search_point, end_point, relevant_layers, prev_octant, o, true, ppo_id, begin_point, end_point);
				}
				else {
					connection = _can_path(prev_octant->search_point, o->origin, relevant_layers, prev_octant, o, false, ppo_id, begin_point, end_point);
				}

				i++;
			}
			Octant* valid_prev_octant = o->prev_octants[i - 1];//r[i - 1];

			//r.release();
			
			//clear previous octants
			o->prev_octants.clear();


			//if no connection can be made to this octant, skip to the next most viable octant
			if (connection == -1) {
				_debug_print(vformat("un-passing octant %d", o->id));
				o->open_pass -= 1; // mark the octant as no longer in the open list so that another path to it may be made to it from a different neighbouring octant (won't this cause an infinite loop?, (probably not since once all neighbouring octants have tried to path to this octant, it will no longer show up within the open_list ))
				continue;
			}
			else {
				//set to valid prev octant
				o->prev_octant = valid_prev_octant;

				//set search point that will be used as the entrance to this octant
				Point* search_point;
				points.lookup(connection, search_point);
				o->search_point = search_point;

				//once it has been pathed to sucessfully:
				o->closed_pass = oct_pass; // Mark the octant as closed

			}


		}
		else {
			//Mark the begin_octant as closed
			o->closed_pass = oct_pass;
		}

		//this means the end point was reached as well,  
		if (o == end_octant) {
			found_route = true;
			break;


		}


		_debug_print(vformat("testing neighbors of octant %d", o->id));


		for (OAHashMap<int64_t, Octant*>::Iterator it = o->neighbours.iter(); it.valid; it = o->neighbours.next_iter(it)) {
			Octant* oe = *(it.value); // The neighbour octant

			//make sure parallel layers are supported within coords of the octant
			// or if *relevant_layers is 0 then use all octants
			supported = relevant_layers == 0 || (relevant_layers & oe->nav_layers) > 0;

			//check if the octant can be pathed to from the current position, using only points from o and oe




			//_debug_print(vformat("		testing neighbor %d: supported is %s, closed is %s",oe->id,  supported?"T":"F",(oe->closed_pass == oct_pass) ? "T" : "F"));

			if (oe->closed_pass == oct_pass || !supported) {
				continue;
			}




			real_t tentative_g_score = o->g_score + _compute_octant_cost(o->id, oe->id) * oe->weight_scale;



			bool new_octant = false;

			if (oe->open_pass != oct_pass) { // The octant wasn't inside the open list.
				oe->open_pass = oct_pass;
				oct_open_list.push_back(oe);
				new_octant = true;
				//clear prev_octants from previous pathing calls
				oe->prev_octants.clear();

				_debug_print(vformat("===new oct neighbor %d", oe->id));
			}
			else if (tentative_g_score >= oe->g_score) { // The new path is worse than the previous.
				continue;
			}

			//multiple octants can fit into the prev_octants because not all are valid connections
			//insert less viable octants (in comparison to this octant) to be path tested first
			oe->prev_octants.insert(0, o);



			oe->g_score = tentative_g_score;
			oe->f_score = oe->g_score + _estimate_octant_cost(oe->id, end_octant->id);


			//if (o->neighbours.has(end_octant->id)) {
			_debug_print(vformat("~~~testing neighbor %d, g_score is %d, f_score is %d", oe->id, tentative_g_score, oe->f_score));
			//}

			if (new_octant) { // The position of the new points is already known.
				oct_sorter.push_heap(0, oct_open_list.size() - 1, 0, oe, oct_open_list.ptr());
			}
			else {
				oct_sorter.push_heap(0, oct_open_list.find(oe), 0, oe, oct_open_list.ptr());
			}
		}
	}


	_debug_print(vformat("found_route %s", found_route ? "T" : "F"));

	return found_route;
}

int AStar3D::_can_path(Point* begin_point, Point* end_point, int32_t relevant_layers, Octant* begin_octant, Octant* end_octant, bool reach_end_point, int64_t prev_octant_id, Point* absolute_begin_point, Point* absolute_end_point) {
	//prev_octant_id is the id of the octant before the begin octant
	int64_t found_point = -1;

	//if only 1 point in the octant, just check if that point is disabled or if it has no neighbors before trying to path to it
	if (end_octant->points.get_num_elements() == 1) {
		for (OAHashMap<int64_t, Point*>::Iterator it = end_octant->points.iter(); it.valid; it = end_octant->points.next_iter(it)) {
			Point* x = *(it.value);
			
			if (!x->enabled) {
				return -1;
			}
			if (x->neighbors.get_num_elements() == 0) {
				return -1;
			}
		}
	}


	//set absolute scores if this is the very first point
	if (begin_point == absolute_begin_point) {
		begin_point->abs_g_score = 0;
		begin_point->abs_f_score = _estimate_cost(begin_point->id, absolute_end_point->id);
	}

	// first try pathing with a straight line to the end point:
	// straight paths may traverse octants that are not in the main octant path defined in _octants_solve, but this can be circumvented when the path is complete
	if (!function_source_id.is_null()) {
		Vector<int64_t> straight_path = _get_straight_line(begin_point->id, end_point->id);
		const int64_t* r = straight_path.ptr();
		int size = straight_path.size();

		Point* prev_p = begin_point;



		//i skips begin point 
		for (int i = 1; i < size; i++) {

			int64_t p_id = r[i];
			int64_t prev_p_id = r[i - 1];

			

			//check if point exists, 
			Point* p;


			if (!points.lookup(p_id, p)) {
				break;
			}

			//check if point is connected to previous point
			Segment s(prev_p_id, p_id);
			const HashSet<Segment, Segment>::Iterator element = segments.find(s);


			bool connected = element && (element->direction & s.direction) == s.direction;

			if (!connected) {
				break;
			}

			//is supported by layers,
			bool supported = relevant_layers == 0 || (relevant_layers & p->nav_layers) > 0;

			//not disabled, and not of a modified weight scale
			if (!p->enabled || !supported || p->weight_scale != real_t(1)) {
				break;
			}


			//the true scores relative to the begin and end points initially defined within _solve()
			p->abs_g_score = prev_p->abs_g_score + _compute_cost(p->id, prev_p->id) * p->weight_scale;
			p->abs_f_score = _estimate_cost(p->id, absolute_end_point->id);

			//closer to end_point, or same distance to end_point but closer to begin_point
			if (closest_point_of_last_pathing_call == nullptr || closest_point_of_last_pathing_call->abs_f_score > p->abs_f_score || (closest_point_of_last_pathing_call->abs_f_score >= p->abs_f_score && closest_point_of_last_pathing_call->abs_g_score > p->abs_g_score)) {
				closest_point_of_last_pathing_call = p;
			}



			//point back to the previous octant
			//in this case, when the point is not within the begin_octant, you can't point to prev_octant_id since, in the back trace loop (in get_point_path() and get_id_path()), the iteration to searching for the previous octant only happens when the previous octant is reached.
			if (p->octant != begin_octant) {
				p->octant_source_prev_point.set(begin_octant->id, prev_p);


				if (p->octant == end_octant) {
					//try to continue the loop in order to reach the end point if true 
					if (reach_end_point) {
						if (p == end_point) {
							found_point = p->id;

							break;
						}
					}
					else {
						found_point = p->id;

						break;
					}
				}


			}
			else {

				_debug_print(vformat("p_id %d, of octant %d, points to prev octant %d, points back to point %d.", p->id, p->octant->id, prev_octant_id, prev_p->id));



				p->octant_source_prev_point.set(prev_octant_id, prev_p);
			}






			prev_p = p;


		}

	}
	else {
		_debug_print("Not using straight paths");
	}


	//if no straight path
	if (found_point == -1) {
		pass++;

		LocalVector<int64_t> octants_list;
		octants_list.push_back(begin_octant->id);
		octants_list.push_back(end_octant->id);






		LocalVector<Point*> open_list;
		SortArray<Point*, SortPoints> sorter;

		begin_point->g_score = 0;
		begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);




		open_list.push_back(begin_point);



		while (!open_list.is_empty()) {
			Point* p = open_list[0]; // The currently processed point


			//closer to end_point, or same distance to end_point but closer to begin_point
			if (closest_point_of_last_pathing_call == nullptr || closest_point_of_last_pathing_call->abs_f_score > p->abs_f_score || (closest_point_of_last_pathing_call->abs_f_score >= p->abs_f_score && closest_point_of_last_pathing_call->abs_g_score > p->abs_g_score)) {
				closest_point_of_last_pathing_call = p;
			}



			if (p != begin_point) {
				_debug_print(vformat("p_id %d, of octant %d, points to prev octant %d, points back to point %d.", p->id, p->octant->id, prev_octant_id, p->prev_point->id));
			}



			if (p->octant == end_octant) {
				//try to reach the end point if true 
				if (reach_end_point) {
					if (p == end_point) {
						found_point = p->id;
						break;
					}
				}
				else {
					found_point = p->id;
					break;
				}



			}






			sorter.pop_heap(0, open_list.size(), open_list.ptr()); // Remove the current point from the open list
			open_list.remove_at(open_list.size() - 1);
			p->closed_pass = pass; // Mark the point as closed










			for (OAHashMap<int64_t, Point*>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
				Point* e = *(it.value); // The neighbour point

				//make sure parallel layers are supported
				// or if *relevant_layers is 0 then use all points
				bool supported = relevant_layers == 0 || (relevant_layers & e->nav_layers) > 0;


				if (!e->enabled || e->closed_pass == pass || !supported || !(octants_list.find(e->octant->id) != -1)) {
					continue;
				}

				real_t tentative_g_score = p->g_score + _compute_cost(p->id, e->id) * e->weight_scale;

				bool new_point = false;

				if (e->open_pass != pass) { // The point wasn't inside the open list.
					e->open_pass = pass;
					open_list.push_back(e);
					new_point = true;
				}
				else if (tentative_g_score >= e->g_score) { // The new path is worse than the previous.
					continue;
				}
				//point back to the previous octant
				if (e->octant == end_octant) {
					e->octant_source_prev_point.set(begin_octant->id, p);
				}
				else {
					e->octant_source_prev_point.set(prev_octant_id, p);
				}


				e->prev_point = p;

				e->g_score = tentative_g_score;
				e->f_score = e->g_score + _estimate_cost(e->id, end_point->id);


				//the true scores relative to the begin and end points initially defined within _solve()
				e->abs_g_score = p->abs_g_score + _compute_cost(p->id, e->id) * e->weight_scale;
				e->abs_f_score = _estimate_cost(p->id, absolute_end_point->id);





				if (new_point) { // The position of the new points is already known.
					sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptr());
				}
				else {
					sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptr());
				}
			}
		}
		if (found_point != -1) {
			_debug_print(vformat("++found_point %d, of end octant %d, from begin octant %d and point %d.", found_point, end_octant->id, begin_octant->id, begin_point->id));
		}
		else {
			_debug_print(vformat("--did not find point to end octant %d, from begin octant %d and point %d.", end_octant->id, begin_octant->id, begin_point->id));
		}
	}

	_debug_print("broke");
	return found_point;
}


bool AStar3D::_solve(Point *begin_point, Point *end_point, int32_t relevant_layers, bool use_octants) {

	id_path_of_last_pathing_call.clear();
	point_path_of_last_pathing_call.clear();
	closest_point_of_last_pathing_call = nullptr;


	if (use_octants) {
		return _octants_solve(begin_point, end_point, relevant_layers);
	}

	pass++;

	if (!end_point->enabled) {
		return false;
	}

	bool found_route = false;

	LocalVector<Point *> open_list;
	SortArray<Point *, SortPoints> sorter;

	begin_point->g_score = 0;
	begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);

	begin_point->abs_g_score = 0;
	begin_point->abs_f_score = _estimate_cost(begin_point->id, end_point->id);

	open_list.push_back(begin_point);

	while (!open_list.is_empty()) {
		Point *p = open_list[0]; // The currently processed point.


		//closer to end_point, or same distance to end_point but closer to begin_point
		if (closest_point_of_last_pathing_call == nullptr || closest_point_of_last_pathing_call->abs_f_score > p->abs_f_score || (closest_point_of_last_pathing_call->abs_f_score >= p->abs_f_score && closest_point_of_last_pathing_call->abs_g_score > p->abs_g_score)) {
			closest_point_of_last_pathing_call = p;
		}

		if (p == end_point) {
			found_route = true;
			break;
		}

		sorter.pop_heap(0, open_list.size(), open_list.ptr()); // Remove the current point from the open list.
		open_list.remove_at(open_list.size() - 1);
		p->closed_pass = pass; // Mark the point as closed.

		for (OAHashMap<int64_t, Point *>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
			Point *e = *(it.value); // The neighbor point.

			//make sure nav layers are supported
			// or if *relevant_layers is 0 then use all points
			bool supported = relevant_layers == 0 || (relevant_layers & e->nav_layers) > 0;


			if (!e->enabled || e->closed_pass == pass || !supported) {
				continue;
			}

			real_t tentative_g_score = p->g_score + _compute_cost(p->id, e->id) * e->weight_scale;

			bool new_point = false;

			if (e->open_pass != pass) { // The point wasn't inside the open list.
				e->open_pass = pass;
				open_list.push_back(e);
				new_point = true;
			} else if (tentative_g_score >= e->g_score) { // The new path is worse than the previous.
				continue;
			}

			e->prev_point = p;
			e->g_score = tentative_g_score;
			e->f_score = e->g_score + _estimate_cost(e->id, end_point->id);

			e->abs_g_score = tentative_g_score;
			e->abs_f_score = e->f_score - e->g_score;

			if (new_point) { // The position of the new points is already known.
				sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptr());
			} else {
				sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptr());
			}
		}
	}

	return found_route;
}

void AStar3D::_debug_print(Variant v)
{
	if (debug_mode_active){
		print_line(v);
	}
	
}

real_t AStar3D::_estimate_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_estimate_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	Point *from_point;
	bool from_exists = points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_from_id));

	Point *to_point;
	bool to_exists = points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t AStar3D::_estimate_octant_cost(int64_t o_from_id, int64_t o_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_estimate_octant_cost, o_from_id, o_to_id, scost)) {
		return scost;
	}

	Octant* from_octant;
	bool from_exists = octants.lookup(o_from_id, from_octant);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Octant with id: %d doesn't exist.", o_from_id));

	Octant* to_octant;
	bool to_exists = octants.lookup(o_to_id, to_octant);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Octant with id: %d doesn't exist.", o_to_id));

	return from_octant->pos.distance_to(to_octant->pos);
}

real_t AStar3D::_compute_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_compute_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	Point *from_point;
	bool from_exists = points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_from_id));

	Point *to_point;
	bool to_exists = points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t AStar3D::_compute_octant_cost(int64_t o_from_id, int64_t o_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_compute_octant_cost, o_from_id, o_to_id, scost)) {
		return scost;
	}

	Octant* from_octant;
	bool from_exists = octants.lookup(o_from_id, from_octant);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Octant with id: %d doesn't exist.", o_from_id));

	Octant* to_octant;
	bool to_exists = octants.lookup(o_to_id, to_octant);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Octant with id: %d doesn't exist.", o_to_id));

	return from_octant->pos.distance_to(to_octant->pos);
}

Vector<Vector3> AStar3D::get_point_path(int64_t p_from_id, int64_t p_to_id, int32_t relevant_layers, bool use_octants) {
	Point *a;
	bool from_exists = points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<Vector3>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_from_id));

	Point *b;
	bool to_exists = points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<Vector3>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));


	if (use_octants) {
		ERR_FAIL_COND_V_MSG(a->octant == nullptr, Vector<Vector3>(), vformat("Can't get point path. Point with id: %d isn't attached to an octant.", p_from_id));
		ERR_FAIL_COND_V_MSG(b->octant == nullptr, Vector<Vector3>(), vformat("Can't get point path. Point with id: %d isn't attached to an octant.", p_to_id));

	}

	if (a == b) {
		Vector<Vector3> ret;
		ret.push_back(a->pos);
		return ret;
	}

	Point *begin_point = a;
	Point *end_point = b;

	Octant* begin_octant = begin_point->octant;
	Octant* end_octant = end_point->octant;

	//can't use _octants_solve if points are within the same octant
	if (begin_octant == end_octant) {
		use_octants = false;
	}

	ERR_FAIL_INDEX_V(relevant_layers, ((1 << 31) - 1), Vector<Vector3>());

	bool found_route = _solve(begin_point, end_point, relevant_layers, use_octants);
	if (!found_route) {
		if (closest_point_of_last_pathing_call == nullptr) {
			return Vector<Vector3>();
		}


		//use closest point instead
		end_point = closest_point_of_last_pathing_call;

		WARN_PRINT(vformat("closest_point_of_last_pathing_call: %d,%d,%d .", closest_point_of_last_pathing_call->pos.x, closest_point_of_last_pathing_call->pos.y, closest_point_of_last_pathing_call->pos.z));
	}

	Point *p = end_point;
	int64_t pc = 1; // Begin point
	if (use_octants) {
		Octant* end_octant = end_point->octant;
		Octant* o = end_octant;
		int oc = 1; // Begin octant




		while (p != begin_point) {
			oc++;

			Octant* po = o->prev_octant;
			int64_t po_id = -1;
			if (po != nullptr) {
				po_id = po->id;
			}
			//might path over octants not in the octant path, so `p->octant != po` is used instead of `p->octant == o`
			while (p->octant != po && p != begin_point) {
				pc++;
				//find the prev point that is in the direction of the previous octant
				Point* pp;
				bool pp_exists = p->octant_source_prev_point.lookup(po_id, pp);
				p->octant_source_prev_point.clear();
				_debug_print(vformat("in p %d pp_exists %s, p->octant = %d, o_id = %d, po_id = %d", p->id, pp_exists ? "T" : "F", p->octant->id, o->id, po_id));

				CRASH_COND_MSG(!pp_exists, "path failed");

				p->prev_point = pp;
				p = pp;
			}



			o = po;
		}


	}
	else {

		while (p != begin_point) {
			pc++;
			p = p->prev_point;
		}



	}

	Vector<Vector3> path;
	path.resize(pc);

	

	{
		Vector3* w = path.ptrw();


		Point* p2 = end_point;
		int idx = pc - 1;
		int removed_p_idx = 0;

		while (p2 != begin_point) {
			w[idx--] = p2->pos;



			if (use_octants) {
				//remove unneccessary points in the path caused by using octants
				if (p2 != begin_point && p2->prev_point != begin_point) {
					Point* skip_point = p2->prev_point->prev_point;

					//check if connected to p2
					Segment s(skip_point->id, p2->id);
					HashSet<Segment, Segment>::Iterator element = segments.find(s);
					bool connected = element && (element->direction & s.direction) == s.direction;

					if (connected) {
						p2 = skip_point;
						removed_p_idx++;
					}
					else {
						p2 = p2->prev_point;
					}
				}
				else {
					p2 = p2->prev_point;
				}

			}
			else {
				p2 = p2->prev_point;
			}

		}




		w[idx] = p2->pos; // Assign first

		_debug_print(vformat("removed_p_idx %d, pc %d", removed_p_idx, pc));
		if (removed_p_idx > 0) {
			for (int i = 0; i < pc - removed_p_idx; i++) {
				w[i] = w[i + removed_p_idx];
			};

			//w.release();

			path.resize(pc - removed_p_idx);
		}






	}

	if (!found_route) {

		point_path_of_last_pathing_call = path;
		return Vector<Vector3>();
	}


	return path;
}

Vector<int64_t> AStar3D::get_id_path(int64_t p_from_id, int64_t p_to_id, int32_t relevant_layers, bool use_octants) {
	Point *a;
	bool from_exists = points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_from_id));

	Point *b;
	bool to_exists = points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_to_id));

	if (use_octants) {
		ERR_FAIL_COND_V_MSG(a->octant == nullptr, Vector<int64_t>(), vformat("Can't get point path. Point with id: %d isn't attached to an octant.", p_from_id));
		ERR_FAIL_COND_V_MSG(b->octant == nullptr, Vector<int64_t>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));

	}

	if (a == b) {
		Vector<int64_t> ret;
		ret.push_back(a->id);
		return ret;
	}

	Point *begin_point = a;
	Point *end_point = b;

	Octant* begin_octant = begin_point->octant;
	Octant* end_octant = end_point->octant;

	//can't use _octants_solve if points are within the same octant
	if (begin_octant == end_octant) {
		use_octants = false;
	}

	ERR_FAIL_INDEX_V(relevant_layers, ((1 << 31) - 1), Vector<int64_t>());

	bool found_route = _solve(begin_point, end_point, relevant_layers, use_octants);
	if (!found_route) {
		if (closest_point_of_last_pathing_call == nullptr) {
			return Vector<int64_t>();
		}

		//use closest point instead
		end_point = closest_point_of_last_pathing_call;
		WARN_PRINT(vformat("closest_point_of_last_pathing_call: %d,%d,%d .", closest_point_of_last_pathing_call->pos.x, closest_point_of_last_pathing_call->pos.y, closest_point_of_last_pathing_call->pos.z));
	}

	Point *p = end_point;
	int64_t pc = 1; // Begin point

	if (use_octants) {
		Octant* end_octant = end_point->octant;
		Octant* o = end_octant;
		int oc = 1; // Begin octant




		while (p != begin_point) {
			oc++;

			Octant* po = o->prev_octant;
			int64_t po_id = -1;
			if (po != nullptr) {
				po_id = po->id;
			}
			//might path over octants not in the octant path, so `p->octant != po` is used instead of `p->octant == o`
			while (p->octant != po && p != begin_point) {
				pc++;
				//find the prev point that is in the direction of the previous octant
				Point* pp;
				bool pp_exists = p->octant_source_prev_point.lookup(po_id, pp);
				p->octant_source_prev_point.clear();

				CRASH_COND_MSG(!pp_exists, "path failed");

				p->prev_point = pp;
				p = pp;
			}



			o = po;
		}


	}
	else {
		while (p != begin_point) {
			pc++;
			p = p->prev_point;
		}
	}

	Vector<int64_t> path;
	path.resize(pc);

	

	{
		int64_t* w = path.ptrw();



		p = end_point;
		int idx = pc - 1;
		int removed_p_idx = 0;

		while (p != begin_point) {
			w[idx--] = p->id;

			if (use_octants) {
				//remove unneccessary points in the path caused by using octants
				if (p != begin_point && p->prev_point != begin_point) {
					Point* skip_point = p->prev_point->prev_point;

					//check if connected to p
					Segment s(skip_point->id, p->id);
					HashSet<Segment, Segment>::Iterator element = segments.find(s);
					bool connected = element  && (element->direction & s.direction) == s.direction;

					if (connected) {
						p = skip_point;
						removed_p_idx++;
					}
					else {
						p = p->prev_point;
					}
				}
				else {
					p = p->prev_point;
				}

			}
			else {
				p = p->prev_point;
			}
		}


		w[idx] = p->id; // Assign first

		

		_debug_print(vformat("removed_p_idx %d, pc %d", removed_p_idx, pc));
		if (removed_p_idx > 0) {
			for (int i = 0; i < pc - removed_p_idx; i++) {
				w[i] = w[i + removed_p_idx];
			};
			//w.release();

			path.resize(pc - removed_p_idx);
		}



	}

	if (!found_route) {
		id_path_of_last_pathing_call = path;

		return Vector<int64_t>();

	}

	return path;
}


//get path of last call to get_id_path if the path_fails
Vector<int64_t> AStar3D::get_proximity_id_path_of_last_pathing_call() {
	return id_path_of_last_pathing_call;
}
//get path of last call to get_point_path if the path_fails
Vector<Vector3> AStar3D::get_proximity_point_path_of_last_pathing_call() {
	return point_path_of_last_pathing_call;
}

void AStar3D::set_point_disabled(int64_t p_id, bool p_disabled) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set if point is disabled. Point with id: %d doesn't exist.", p_id));

	p->enabled = !p_disabled;
}

bool AStar3D::is_point_disabled(int64_t p_id) const {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get if point is disabled. Point with id: %d doesn't exist.", p_id));

	return !p->enabled;
}



void AStar3D::set_point_layer(int64_t p_id, int64_t layer_index, bool l_enabled)
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point layer index. Point with id: %d doesn't exist.", p_id));


	ERR_FAIL_INDEX(layer_index, 31);

	uint32_t layers = p->nav_layers;

	if (l_enabled) {
		p->nav_layers = layers | (1 << layer_index);
	}
	else {
		p->nav_layers = layers & (~(1 << layer_index));
	}




	//changes to layers results in the removal of octant point is attached to, since it is no longer certain what layers are supported alter the change without looping through all the octant's points:
	//remove the octant if any
	if (p->octant != nullptr) {
		int64_t o_id = p->octant->id;
		remove_octant(o_id);

	}

}

void AStar3D::set_point_layers_value(int64_t p_id, int64_t p_layers)
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point layer value. Point with id: %d doesn't exist.", p_id));
	ERR_FAIL_INDEX_MSG(p_layers, ((1 << 31) - 1), vformat("Can't add a point with layers value less than 0 or more than 2^31 - 1: %d.", p_layers));


	

	

	p->nav_layers = p_layers;
	




	//changes to layers results in the removal of octant point is attached to, since it is no longer certain what layers are supported alter the change without looping through all the octant's points:
	//remove the octant if any
	if (p->octant != nullptr) {
		int64_t o_id = p->octant->id;
		remove_octant(o_id);

	}

}

bool AStar3D::get_point_layer(int64_t p_id, int64_t layer_index) const
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get if point supports layer index. Point with id: %d doesn't exist.", p_id));

	uint32_t layers = p->nav_layers;
	ERR_FAIL_INDEX_V(layer_index, 31, false);
	return (layers & (1 << layer_index)) > 0;
}

int32_t AStar3D::get_point_layers_value(int64_t p_id) const
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get the point's layers value. Point with id: %d doesn't exist.", p_id));

	return p->nav_layers;

}

//returns false if fail
bool AStar3D::set_straight_line_function(Object* p_obj, const StringName& draw_straight_line_f_name) {

	ERR_FAIL_NULL_V(p_obj, false);


	ERR_FAIL_COND_V_MSG(draw_straight_line_f_name == "", false, "No defined Straight Line Function");


	ERR_FAIL_COND_V_MSG(!p_obj->has_method(draw_straight_line_f_name), false, vformat("function %s does not exist within object", draw_straight_line_f_name));


	ObjectID id = p_obj->get_instance_id();
	//this check might be redundant
	ERR_FAIL_COND_V(id.is_null(), false);

	int64_t test_from = 0;
	int64_t test_to = 1;

	//test the function to ensure it works properly and returns the correct var type:
	if (!points.has(test_from) || !points.has(test_to)) {
		ERR_FAIL_V_MSG(false,"astar points with ids 0 and 1 required in order to test that this function returns the correct variable type");
	}

	Array p_args;
	p_args.append(test_from);
	p_args.append(test_to);

	//how do I check the number of parameters of the straight_line_function is 2?

	Variant result = p_obj->callv(draw_straight_line_f_name, p_args);

	ERR_FAIL_COND_V_MSG(result.get_type() != Variant::PACKED_INT64_ARRAY, false, vformat("straight line function %s, does not return a PACKED_INT64_ARRAY", draw_straight_line_f_name));


	straight_line_function = draw_straight_line_f_name;
	function_source_id = id;

	return true;
}

PackedInt64Array AStar3D::_get_straight_line(int64_t from_p_id, int64_t to_p_id) {
	

	

	ERR_FAIL_COND_V(function_source_id.is_null(), PackedInt64Array());

	Object* obj = ObjectDB::get_instance(function_source_id);

	ERR_FAIL_COND_V(!obj, PackedInt64Array());

	
	Array p_args;
	p_args.append(from_p_id);
	p_args.append(to_p_id);

	Variant result = obj->callv(straight_line_function, p_args);

	//can i just return the result even though it is a Variant?
	return result;

}

void AStar3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_debug_mode", "is_active"), &AStar3D::set_debug_mode);
	ClassDB::bind_method(D_METHOD("get_available_point_id"), &AStar3D::get_available_point_id);
	ClassDB::bind_method(D_METHOD("add_point", "id", "position", "weight_scale", "point_layers"), &AStar3D::add_point, DEFVAL(1.0), DEFVAL(0));
	ClassDB::bind_method(D_METHOD("add_octant", "id", "pool_points", "pos", "center_point"), &AStar3D::add_octant);
	ClassDB::bind_method(D_METHOD("debug_octant", "id"), &AStar3D::debug_octant);
	ClassDB::bind_method(D_METHOD("get_point_octant_id", "id"), &AStar3D::get_point_octant_id);
	ClassDB::bind_method(D_METHOD("get_octant_ids"), &AStar3D::get_octant_ids);
	ClassDB::bind_method(D_METHOD("remove_octant", "id"), &AStar3D::remove_octant);

	ClassDB::bind_method(D_METHOD("append_as_bulk_array", "pool_points", "max_connections", "pool_connections"), &AStar3D::append_as_bulk_array);
	ClassDB::bind_method(D_METHOD("set_as_bulk_array", "pool_points", "max_connections", "pool_connections"), &AStar3D::set_as_bulk_array);

	ClassDB::bind_method(D_METHOD("get_point_position", "id"), &AStar3D::get_point_position);
	ClassDB::bind_method(D_METHOD("set_point_position", "id", "position"), &AStar3D::set_point_position);
	ClassDB::bind_method(D_METHOD("get_point_weight_scale", "id"), &AStar3D::get_point_weight_scale);
	ClassDB::bind_method(D_METHOD("set_point_weight_scale", "id", "weight_scale"), &AStar3D::set_point_weight_scale);
	ClassDB::bind_method(D_METHOD("remove_point", "id"), &AStar3D::remove_point);
	ClassDB::bind_method(D_METHOD("has_point", "id"), &AStar3D::has_point);
	ClassDB::bind_method(D_METHOD("get_point_connections", "id"), &AStar3D::get_point_connections);
	ClassDB::bind_method(D_METHOD("get_point_ids"), &AStar3D::get_point_ids);

	ClassDB::bind_method(D_METHOD("set_straight_line_function", "p_obj", "draw_straight_line_f_name"), &AStar3D::set_straight_line_function);

	ClassDB::bind_method(D_METHOD("set_point_disabled", "id", "disabled"), &AStar3D::set_point_disabled, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("is_point_disabled", "id"), &AStar3D::is_point_disabled);

	ClassDB::bind_method(D_METHOD("set_point_layer", "id", "layer_index", "l_enabled"), &AStar3D::set_point_layer, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("set_point_layers_value", "id", "layer_index"), &AStar3D::set_point_layers_value);
	ClassDB::bind_method(D_METHOD("get_point_layer", "id", "layer_index"), &AStar3D::get_point_layer);
	ClassDB::bind_method(D_METHOD("get_point_layers_value", "id"), &AStar3D::get_point_layers_value);


	ClassDB::bind_method(D_METHOD("connect_points", "id", "to_id", "bidirectional"), &AStar3D::connect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("connect_octants", "id", "to_id", "bidirectional"), &AStar3D::connect_octants, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("disconnect_points", "id", "to_id", "bidirectional"), &AStar3D::disconnect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("are_points_connected", "id", "to_id", "bidirectional"), &AStar3D::are_points_connected, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("are_octants_connected", "id", "to_id", "bidirectional"), &AStar3D::are_octants_connected, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("get_point_count"), &AStar3D::get_point_count);
	ClassDB::bind_method(D_METHOD("get_point_capacity"), &AStar3D::get_point_capacity);
	ClassDB::bind_method(D_METHOD("reserve_space", "num_nodes"), &AStar3D::reserve_space);
	ClassDB::bind_method(D_METHOD("clear"), &AStar3D::clear);

	ClassDB::bind_method(D_METHOD("get_closest_point", "to_position", "include_disabled", "relevant_layers"), &AStar3D::get_closest_point, DEFVAL(false), DEFVAL(0));
	ClassDB::bind_method(D_METHOD("get_closest_position_in_segment", "to_position"), &AStar3D::get_closest_position_in_segment);

	ClassDB::bind_method(D_METHOD("get_point_path", "p_from_id", "p_to_id", "relevant_layers", "use_octants"), &AStar3D::get_point_path, DEFVAL(0), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("get_id_path", "p_from_id", "p_to_id", "relevant_layers", "use_octants"), &AStar3D::get_id_path, DEFVAL(0), DEFVAL(false));

	ClassDB::bind_method(D_METHOD("get_proximity_id_path_of_last_pathing_call"), &AStar3D::get_proximity_id_path_of_last_pathing_call);
	ClassDB::bind_method(D_METHOD("get_proximity_point_path_of_last_pathing_call"), &AStar3D::get_proximity_point_path_of_last_pathing_call);


	ClassDB::bind_method(D_METHOD("_get_straight_line", "from_id", "to_id"), &AStar3D::_get_straight_line);

	GDVIRTUAL_BIND(_debug_print, "v")
	GDVIRTUAL_BIND(_estimate_cost, "from_id", "to_id")
	GDVIRTUAL_BIND(_compute_cost, "from_id", "to_id")
	GDVIRTUAL_BIND(_estimate_octant_cost, "from_id", "to_id")
	GDVIRTUAL_BIND(_compute_octant_cost, "from_id", "to_id")
}

AStar3D::~AStar3D() {
	clear();
}

/////////////////////////////////////////////////////////////

int64_t AStar2D::get_available_point_id() const {
	return astar.get_available_point_id();
}

void AStar2D::add_point(int64_t p_id, const Vector2 &p_pos, real_t p_weight_scale) {
	astar.add_point(p_id, Vector3(p_pos.x, p_pos.y, 0), p_weight_scale);
}

Vector2 AStar2D::get_point_position(int64_t p_id) const {
	Vector3 p = astar.get_point_position(p_id);
	return Vector2(p.x, p.y);
}

void AStar2D::set_point_position(int64_t p_id, const Vector2 &p_pos) {
	astar.set_point_position(p_id, Vector3(p_pos.x, p_pos.y, 0));
}

real_t AStar2D::get_point_weight_scale(int64_t p_id) const {
	return astar.get_point_weight_scale(p_id);
}

void AStar2D::set_point_weight_scale(int64_t p_id, real_t p_weight_scale) {
	astar.set_point_weight_scale(p_id, p_weight_scale);
}

void AStar2D::remove_point(int64_t p_id) {
	astar.remove_point(p_id);
}

bool AStar2D::has_point(int64_t p_id) const {
	return astar.has_point(p_id);
}

Vector<int64_t> AStar2D::get_point_connections(int64_t p_id) {
	return astar.get_point_connections(p_id);
}

PackedInt64Array AStar2D::get_point_ids() {
	return astar.get_point_ids();
}

void AStar2D::set_point_disabled(int64_t p_id, bool p_disabled) {
	astar.set_point_disabled(p_id, p_disabled);
}

bool AStar2D::is_point_disabled(int64_t p_id) const {
	return astar.is_point_disabled(p_id);
}

void AStar2D::connect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional) {
	astar.connect_points(p_id, p_with_id, p_bidirectional);
}

void AStar2D::disconnect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional) {
	astar.disconnect_points(p_id, p_with_id, p_bidirectional);
}

bool AStar2D::are_points_connected(int64_t p_id, int64_t p_with_id, bool p_bidirectional) const {
	return astar.are_points_connected(p_id, p_with_id, p_bidirectional);
}

int64_t AStar2D::get_point_count() const {
	return astar.get_point_count();
}

int64_t AStar2D::get_point_capacity() const {
	return astar.get_point_capacity();
}

void AStar2D::clear() {
	astar.clear();
}

void AStar2D::reserve_space(int64_t p_num_nodes) {
	astar.reserve_space(p_num_nodes);
}

int64_t AStar2D::get_closest_point(const Vector2 &p_point, bool p_include_disabled) const {
	return astar.get_closest_point(Vector3(p_point.x, p_point.y, 0), p_include_disabled);
}

Vector2 AStar2D::get_closest_position_in_segment(const Vector2 &p_point) const {
	Vector3 p = astar.get_closest_position_in_segment(Vector3(p_point.x, p_point.y, 0));
	return Vector2(p.x, p.y);
}

real_t AStar2D::_estimate_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_estimate_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	AStar3D::Point *from_point;
	bool from_exists = astar.points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_from_id));

	AStar3D::Point *to_point;
	bool to_exists = astar.points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t AStar2D::_compute_cost(int64_t p_from_id, int64_t p_to_id) {
	real_t scost;
	if (GDVIRTUAL_CALL(_compute_cost, p_from_id, p_to_id, scost)) {
		return scost;
	}

	AStar3D::Point *from_point;
	bool from_exists = astar.points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_from_id));

	AStar3D::Point *to_point;
	bool to_exists = astar.points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

Vector<Vector2> AStar2D::get_point_path(int64_t p_from_id, int64_t p_to_id) {
	AStar3D::Point *a;
	bool from_exists = astar.points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<Vector2>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_from_id));

	AStar3D::Point *b;
	bool to_exists = astar.points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<Vector2>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		Vector<Vector2> ret = { Vector2(a->pos.x, a->pos.y) };
		return ret;
	}

	AStar3D::Point *begin_point = a;
	AStar3D::Point *end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return Vector<Vector2>();
	}

	AStar3D::Point *p = end_point;
	int64_t pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	Vector<Vector2> path;
	path.resize(pc);

	{
		Vector2 *w = path.ptrw();

		AStar3D::Point *p2 = end_point;
		int64_t idx = pc - 1;
		while (p2 != begin_point) {
			w[idx--] = Vector2(p2->pos.x, p2->pos.y);
			p2 = p2->prev_point;
		}

		w[0] = Vector2(p2->pos.x, p2->pos.y); // Assign first
	}

	return path;
}

Vector<int64_t> AStar2D::get_id_path(int64_t p_from_id, int64_t p_to_id) {
	AStar3D::Point *a;
	bool from_exists = astar.points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_from_id));

	AStar3D::Point *b;
	bool to_exists = astar.points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, Vector<int64_t>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		Vector<int64_t> ret;
		ret.push_back(a->id);
		return ret;
	}

	AStar3D::Point *begin_point = a;
	AStar3D::Point *end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return Vector<int64_t>();
	}

	AStar3D::Point *p = end_point;
	int64_t pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	Vector<int64_t> path;
	path.resize(pc);

	{
		int64_t *w = path.ptrw();

		p = end_point;
		int64_t idx = pc - 1;
		while (p != begin_point) {
			w[idx--] = p->id;
			p = p->prev_point;
		}

		w[0] = p->id; // Assign first
	}

	return path;
}

bool AStar2D::_solve(AStar3D::Point *begin_point, AStar3D::Point *end_point) {
	astar.pass++;

	if (!end_point->enabled) {
		return false;
	}

	bool found_route = false;

	LocalVector<AStar3D::Point *> open_list;
	SortArray<AStar3D::Point *, AStar3D::SortPoints> sorter;

	begin_point->g_score = 0;
	begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);
	open_list.push_back(begin_point);

	while (!open_list.is_empty()) {
		AStar3D::Point *p = open_list[0]; // The currently processed point.

		if (p == end_point) {
			found_route = true;
			break;
		}

		sorter.pop_heap(0, open_list.size(), open_list.ptr()); // Remove the current point from the open list.
		open_list.remove_at(open_list.size() - 1);
		p->closed_pass = astar.pass; // Mark the point as closed.

		for (OAHashMap<int64_t, AStar3D::Point *>::Iterator it = p->neighbors.iter(); it.valid; it = p->neighbors.next_iter(it)) {
			AStar3D::Point *e = *(it.value); // The neighbor point.

			if (!e->enabled || e->closed_pass == astar.pass) {
				continue;
			}

			real_t tentative_g_score = p->g_score + _compute_cost(p->id, e->id) * e->weight_scale;

			bool new_point = false;

			if (e->open_pass != astar.pass) { // The point wasn't inside the open list.
				e->open_pass = astar.pass;
				open_list.push_back(e);
				new_point = true;
			} else if (tentative_g_score >= e->g_score) { // The new path is worse than the previous.
				continue;
			}

			e->prev_point = p;
			e->g_score = tentative_g_score;
			e->f_score = e->g_score + _estimate_cost(e->id, end_point->id);

			if (new_point) { // The position of the new points is already known.
				sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptr());
			} else {
				sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptr());
			}
		}
	}

	return found_route;
}

void AStar2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_available_point_id"), &AStar2D::get_available_point_id);
	ClassDB::bind_method(D_METHOD("add_point", "id", "position", "weight_scale"), &AStar2D::add_point, DEFVAL(1.0));
	ClassDB::bind_method(D_METHOD("get_point_position", "id"), &AStar2D::get_point_position);
	ClassDB::bind_method(D_METHOD("set_point_position", "id", "position"), &AStar2D::set_point_position);
	ClassDB::bind_method(D_METHOD("get_point_weight_scale", "id"), &AStar2D::get_point_weight_scale);
	ClassDB::bind_method(D_METHOD("set_point_weight_scale", "id", "weight_scale"), &AStar2D::set_point_weight_scale);
	ClassDB::bind_method(D_METHOD("remove_point", "id"), &AStar2D::remove_point);
	ClassDB::bind_method(D_METHOD("has_point", "id"), &AStar2D::has_point);
	ClassDB::bind_method(D_METHOD("get_point_connections", "id"), &AStar2D::get_point_connections);
	ClassDB::bind_method(D_METHOD("get_point_ids"), &AStar2D::get_point_ids);

	ClassDB::bind_method(D_METHOD("set_point_disabled", "id", "disabled"), &AStar2D::set_point_disabled, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("is_point_disabled", "id"), &AStar2D::is_point_disabled);

	ClassDB::bind_method(D_METHOD("connect_points", "id", "to_id", "bidirectional"), &AStar2D::connect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("disconnect_points", "id", "to_id", "bidirectional"), &AStar2D::disconnect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("are_points_connected", "id", "to_id", "bidirectional"), &AStar2D::are_points_connected, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("get_point_count"), &AStar2D::get_point_count);
	ClassDB::bind_method(D_METHOD("get_point_capacity"), &AStar2D::get_point_capacity);
	ClassDB::bind_method(D_METHOD("reserve_space", "num_nodes"), &AStar2D::reserve_space);
	ClassDB::bind_method(D_METHOD("clear"), &AStar2D::clear);

	ClassDB::bind_method(D_METHOD("get_closest_point", "to_position", "include_disabled"), &AStar2D::get_closest_point, DEFVAL(false));
	ClassDB::bind_method(D_METHOD("get_closest_position_in_segment", "to_position"), &AStar2D::get_closest_position_in_segment);

	ClassDB::bind_method(D_METHOD("get_point_path", "from_id", "to_id"), &AStar2D::get_point_path);
	ClassDB::bind_method(D_METHOD("get_id_path", "from_id", "to_id"), &AStar2D::get_id_path);

	GDVIRTUAL_BIND(_estimate_cost, "from_id", "to_id")
	GDVIRTUAL_BIND(_compute_cost, "from_id", "to_id")
}
