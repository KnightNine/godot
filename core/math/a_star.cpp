/*************************************************************************/
/*  a_star.cpp                                                           */
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

#include "a_star.h"

#include "core/math/geometry.h"
#include "core/script_language.h"
#include "scene/scene_string_names.h"

int AStar::get_available_point_id() const {
	if (points.has(last_free_id)) {
		int cur_new_id = last_free_id + 1;
		while (points.has(cur_new_id)) {
			cur_new_id++;
		}
		const_cast<int &>(last_free_id) = cur_new_id;
	}

	return last_free_id;
}

void AStar::add_point(int p_id, const Vector3 &p_pos, real_t p_weight_scale, int p_layers) {
	ERR_FAIL_COND_MSG(p_id < 0, vformat("Can't add a point with negative id: %d.", p_id));
	ERR_FAIL_COND_MSG(p_weight_scale < 0, vformat("Can't add a point with weight scale less than 0.0: %f.", p_weight_scale));
	ERR_FAIL_INDEX_MSG(p_layers, ((1 << 31) - 1), vformat("Can't add a point with layers value less than 0 or more than 2^31 - 1: %d.", p_layers));

	Point *found_pt;
	bool p_exists = points.lookup(p_id, found_pt);

	if (!p_exists) {
		Point *pt = memnew(Point);
		pt->id = p_id;
		pt->pos = p_pos;
		pt->on_empty_edge = false;
		pt->weight_scale = p_weight_scale;
		pt->parallel_support_layers = p_layers;
		pt->prev_point = nullptr;
		pt->prev_point_connected = true;
		pt->is_neighbour = false;
		pt->open_pass = 0;
		pt->closed_pass = 0;
		pt->enabled = true;
		points.set(p_id, pt);
	} else {
		found_pt->pos = p_pos;
		found_pt->weight_scale = p_weight_scale;
	}
}



void AStar::add_empty(int e_id, const PoolVector<int> &pool_points, const PoolVector<int> &pool_edge_points) {
	ERR_FAIL_COND_MSG(e_id < 0, vformat("Can't add a empty with negative id: %d.", e_id));
	Empty *found_em;
	bool e_exists = empties.lookup(e_id, found_em);

	uint32_t parallel_support_layers = 0;
	PoolVector<int>::Read r = pool_points.read();
	int size = pool_points.size();
	ERR_FAIL_COND_MSG(size <= 0, vformat("Can't add a empty zero pool_points: %d.", e_id));
	
	int p_id = r[0];
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	if (p_exists) {
		//use any pool point within points and subtract incongruencies from there
		parallel_support_layers = p->parallel_support_layers;


	}
	
	
	


	//if placed overlapping with an existing empty's points, remove this empty
	bool invalid = false;
	int invalid_type = 0;
	int overlapping_p_id = 0;
	

	if (!e_exists) {
		Empty* em = memnew(Empty);
		em->id = e_id;

		int size = pool_points.size();
		
		for (int i = 0; i < size; i++) {

			int p_id = r[i];
			Point* p;
			bool p_exists = points.lookup(p_id, p);
			if (p_exists) {
				//this will remove layers that aren't supported across all points within the empty
				parallel_support_layers = p->parallel_support_layers & parallel_support_layers;


				//cannot overlap with other empty points
				if (p->empties.size() == 0) {
					p->empties.append(em);
					p->on_empty_edge = false;
					em->points.set(p_id, p);

					

					if (!p->enabled) {
						em->disabled_points.append(p_id);
					}
					if (p->weight_scale != real_t(1)) {
						
						em->weighted_points.append(p_id);
						
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

		if (!invalid){

			size = pool_edge_points.size();
			PoolVector<int>::Read r2 = pool_edge_points.read();
			for (int i = 0; i < size; i++) {

				int p_id = r2[i];
				Point* p;
				bool p_exists = points.lookup(p_id, p);
				if (p_exists) {

					//this will remove layers that aren't supported across all points within the empty
					parallel_support_layers = p->parallel_support_layers & parallel_support_layers;

					//only edges are allowed to overlap with other edges
					if (p->empties.size() == 0 || p->on_empty_edge) {
						p->empties.append(em);
						p->on_empty_edge = true;
						em->edge_points.set(p_id, p);

						if (!p->enabled) {
							em->disabled_points.append(p_id);
						}
						if (p->weight_scale != real_t(1)) {

							em->weighted_points.append(p_id);

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

		}
		

		em->enabled = em->weighted_points.size() == 0  && em->disabled_points.size() == 0;
		em->parallel_support_layers = parallel_support_layers;

		empties.set(e_id, em);
	}
	else {
		//clear old points
		for (OAHashMap<int, Point*>::Iterator it = found_em->points.iter(); it.valid; it = found_em->points.next_iter(it)) {
			Point* p = *it.value;
			p->empties.empty();
			p->on_empty_edge = false;

		}

		found_em->disabled_points.empty();

		found_em->points.clear();
		int size = pool_points.size();
		
		for (int i = 0; i < size; i++) {

			int p_id = r[i];
			Point* p;
			bool p_exists = points.lookup(p_id, p);
			if (p_exists) {
				//this will remove layers that aren't supported across all points within the empty
				parallel_support_layers = p->parallel_support_layers & parallel_support_layers;

				//cannot overlap with other empty points
				if (p->empties.size() == 0) {
					p->empties.append(found_em);
					p->on_empty_edge = false;
					found_em->points.set(p_id, p);

					if (!p->enabled) {
						found_em->disabled_points.append(p_id);
					}
					if (p->weight_scale != real_t(1)) {

						found_em->weighted_points.append(p_id);

					}
				}
				else {
					invalid = true;
					break;
				}

			}
			else {
				invalid = true;
				invalid_type = 1;
				break;
			}

		}

		//clear old edge points
		for (OAHashMap<int, Point*>::Iterator it = found_em->edge_points.iter(); it.valid; it = found_em->edge_points.next_iter(it)) {
			Point* p = *it.value;
			int i = p->empties.find(found_em);
			p->empties.remove(i);
			p->on_empty_edge = false;

		}

		found_em->edge_points.clear();

		if (!invalid) {
			size = pool_edge_points.size();
			PoolVector<int>::Read r2 = pool_edge_points.read();
			for (int i = 0; i < size; i++) {

				int p_id = r2[i];
				Point* p;
				bool p_exists = points.lookup(p_id, p);
				if (p_exists) {
					//this will remove layers that aren't supported across all points within the empty
					parallel_support_layers = p->parallel_support_layers & parallel_support_layers;


					//only edges are allowed to overlap with othwr edges
					if (p->empties.size() == 0 || p->on_empty_edge) {
						p->empties.append(found_em);
						p->on_empty_edge = true;
						found_em->edge_points.set(p_id, p);

						if (!p->enabled) {
							found_em->disabled_points.append(p_id);
						}
						if (p->weight_scale != real_t(1)) {

							found_em->weighted_points.append(p_id);

						}
					}
					else {
						invalid = true;
						break;
					}
				}
				else {
					invalid = true;
					invalid_type = 1;
					break;
				}
			

			}
			
		}

		
		// only enabled when containing no disabled points or weighted points
		found_em->enabled = found_em->weighted_points.size() == 0 && found_em->disabled_points.size() == 0;
		found_em->parallel_support_layers = parallel_support_layers;
	}

	if (invalid) {
		
		remove_empty(e_id);

		//usure how i would go about printing an error without cancelling the function
		if (invalid_type == 1) {
			ERR_FAIL_COND_MSG(invalid, vformat("empty placement of id %d contains points which do not exist and is therefore invalid and has been removed", e_id));
		}
		ERR_FAIL_COND_MSG(invalid, vformat("empty placement of id %d overlaps with another empty at point %d and is therefore invalid and has been removed", e_id, overlapping_p_id));
	}

}
//returns int array 
PoolVector<int> AStar::debug_empty(int e_id) {
	Empty* e;
	bool e_exists = empties.lookup(e_id, e);
	ERR_FAIL_COND_V_MSG(!e_exists, PoolVector<int>(), vformat("Can't debug empty. Empty with id: %d doesn't exist.", e_id));

	//debug_data = [enabled(1 or 0),empty_layers,points_list_type,points_list]

	PoolVector<int> debug_data;
	int enabled = (e->enabled ? 1 : 0);
	debug_data.append(enabled);

	int layers = e->parallel_support_layers;
	debug_data.append(layers);

	if (!e->enabled) {
		
		if (e->weighted_points.size() > 0) {
			debug_data.append(0); // 0 if weighted_points
			PoolVector < int>::Read r = e->weighted_points.read();
			for (int i = 0; i < e->weighted_points.size(); i++) {
				debug_data.append(r[i]);
			}
		}
		else if (e->disabled_points.size() > 0) {
			debug_data.append(1); // 1 if disabled_point
			PoolVector < int>::Read r = e->disabled_points.read();
			for (int i = 0; i < e->disabled_points.size(); i++) {
				debug_data.append(r[i]);
			}
		}
	}
	return debug_data;
	
}
PoolVector<int> AStar::get_point_empty_ids(int p_id) {
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, PoolVector<int>(), vformat("Can't get if point has empty_ids. Point with id: %d doesn't exist.", p_id));

	int size = p->empties.size();
	PoolVector<int> ems;
	ems.resize(size);
	PoolVector<int>::Write w = ems.write();
	PoolVector <Empty*>::Read r = p->empties.read();
	for (int i = 0; i < size; i++) {
		w[i] = r[i]->id;
	}

	return ems;
}
PoolVector<int> AStar::get_empties() {
	PoolVector<int> e_ids;
	for (OAHashMap<int, Empty*>::Iterator it = empties.iter(); it.valid; it = empties.next_iter(it)) {
		int e_id = *it.key;
		e_ids.append(e_id);
	}

	return e_ids;
}



void AStar::remove_empty(int e_id) {
	Empty* e;
	bool e_exists = empties.lookup(e_id, e);
	ERR_FAIL_COND_MSG(!e_exists, vformat("Can't remove empty. Empty with id: %d doesn't exist.", e_id));


	for (OAHashMap<int, Point*>::Iterator it = e->points.iter(); it.valid; it = e->points.next_iter(it)) {
		Point* p = *it.value;
		p->empties.empty();
		p->on_empty_edge = false;

	}

	for (OAHashMap<int, Point*>::Iterator it = e->edge_points.iter(); it.valid; it = e->edge_points.next_iter(it)) {
		Point* p = *it.value;
		int i = p->empties.find(e);
		p->empties.remove(i);
		p->on_empty_edge = false;

	}
	memdelete(e);
	empties.remove(e_id);
	

}

void AStar::append_as_bulk_array(const PoolVector<real_t> &pool_points, int max_connections, const PoolVector<int> &pool_connections)
{
	

	


	
	int size = pool_points.size();

	ERR_FAIL_COND_MSG(size % 6 > 0, vformat("pool_points size lacks data for each point"));

	PoolVector<real_t>::Read r = pool_points.read();
	for (int i = 0; i < size / 6; i++) {
		
		int p_id = r[i * 6 + 0];
		float x = r[i * 6 + 1];
		float y = r[i * 6 + 2];
		float z = r[i * 6 + 3];
		float p_weight_scale = r[i * 6 + 4];
		int p_layers = r[i * 6 + 5];
		Vector3 p_pos = Vector3(x, y, z);
		add_point(p_id, p_pos, p_weight_scale, p_layers);
		
	}

	PoolVector < int>::Read r1 = pool_connections.read();
	size = pool_connections.size();
	int i_mult = max_connections + 1;
	ERR_FAIL_COND_MSG(size % i_mult > 0, vformat("pool_connections size lacks data for each point"));

	for (int i = 0; i < size / i_mult; i++) {

		int p_id = r1[i * i_mult + 0];
		for (int j = 1; j < i_mult; j++) {
			
			int p_with_id = r1[i * i_mult + j];
			if (p_with_id >= 0) {
				connect_points(p_id, p_with_id);
			}
			
		}

		
		

	}

}

// this may be faster than doing it in gdscript
void AStar::set_as_bulk_array(const PoolVector<real_t> &pool_points, int max_connections, const PoolVector<int> &pool_connections )
{

	//wipe all points
	clear();

	int size = pool_points.size();
	ERR_FAIL_COND_MSG(size % 6 > 0, vformat("pool_points size lacks data for each point"));

	PoolVector<real_t>::Read r = pool_points.read();
	for (int i = 0; i < size / 6; i++) {

		int p_id = r[i * 6 + 0];
		float x = r[i * 6 + 1];
		float y = r[i * 6 + 2];
		float z = r[i * 6 + 3];
		float p_weight_scale = r[i * 6 + 4];
		int p_layers = r[i * 6 + 5];

		ERR_FAIL_COND_MSG(p_id < 0, vformat("Can't add a point with negative id: %d.", p_id));
		ERR_FAIL_COND_MSG(p_weight_scale < 0, vformat("Can't add a point with weight scale less than 0.0: %f.", p_weight_scale));
		ERR_FAIL_INDEX_MSG(p_layers, ((1 << 31) - 1), vformat("Can't add a point with layers value less than 0 or more than 2^31 - 1: %d.", p_layers));

		Vector3 p_pos = Vector3(x, y, z);

		Point* pt = memnew(Point);
		pt->id = p_id;
		pt->pos = p_pos;
		pt->weight_scale = p_weight_scale;
		pt->parallel_support_layers = p_layers;
		pt->prev_point = nullptr;
		pt->open_pass = 0;
		pt->closed_pass = 0;
		pt->enabled = true;
		points.set(p_id, pt);



	}

	PoolVector < int>::Read r1 = pool_connections.read();
	size = pool_connections.size();
	int i_mult = max_connections + 1;
	ERR_FAIL_COND_MSG(size % i_mult > 0, vformat("pool_connections size lacks data for each point"));

	for (int i = 0; i < size / i_mult; i++) {

		int p_id = r1[i * i_mult + 0];
		for (int j = 1; j < i_mult; j++) {

			int p_with_id = r1[i * i_mult + j];
			if (p_with_id >= 0) {
				connect_points(p_id, p_with_id);
			}

		}




	}
}

Vector3 AStar::get_point_position(int p_id) const {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, Vector3(), vformat("Can't get point's position. Point with id: %d doesn't exist.", p_id));

	return p->pos;
}

void AStar::set_point_position(int p_id, const Vector3 &p_pos) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point's position. Point with id: %d doesn't exist.", p_id));

	p->pos = p_pos;
}




real_t AStar::get_point_weight_scale(int p_id) const {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, 0, vformat("Can't get point's weight scale. Point with id: %d doesn't exist.", p_id));

	return p->weight_scale;
}

void AStar::set_point_weight_scale(int p_id, real_t p_weight_scale) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point's weight scale. Point with id: %d doesn't exist.", p_id));
	ERR_FAIL_COND_MSG(p_weight_scale < 0, vformat("Can't set point's weight scale less than 0.0: %f.", p_weight_scale));

	p->weight_scale = p_weight_scale;

	// if point is part of an empty, disable the empties it is a part of if weight scale is not equal to 1
	int size = p->empties.size();
	if (size > 0) {
		PoolVector<Empty*>::Read r = p->empties.read();
		for (int i = 0; i < size; i++) {

			
			Empty* e = r[i];

			
			

			if (p->weight_scale != real_t(1)) {
				e->weighted_points.append(p_id);
			}
			else {
				int i = e->weighted_points.find(p_id);
				e->weighted_points.remove(i);

			}


			// only enabled when containing no disabled points
			e->enabled = e->weighted_points.size() == 0 && e->disabled_points.size() == 0;
		}
	}

}

void AStar::remove_point(int p_id) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't remove point. Point with id: %d doesn't exist.", p_id));

	for (OAHashMap<int, Point *>::Iterator it = p->neighbours.iter(); it.valid; it = p->neighbours.next_iter(it)) {
		Segment s(p_id, (*it.key));
		segments.erase(s);

		(*it.value)->neighbours.remove(p->id);
		(*it.value)->unlinked_neighbours.remove(p->id);
	}

	for (OAHashMap<int, Point *>::Iterator it = p->unlinked_neighbours.iter(); it.valid; it = p->unlinked_neighbours.next_iter(it)) {
		Segment s(p_id, (*it.key));
		segments.erase(s);

		(*it.value)->neighbours.remove(p->id);
		(*it.value)->unlinked_neighbours.remove(p->id);
	}

	
	//first fetch all e_ids to be removed
	PoolVector<int> removal_arr;
	PoolVector<Empty*>::Read r = p->empties.read();
	for (int i = 0; i < p->empties.size(); i++) {
		int e_id = r[i]->id;
		removal_arr.append(e_id);

	}

	PoolVector<int>::Read r1 = removal_arr.read();
	for (int i = 0; i < removal_arr.size(); i++) {
		remove_empty(r1[i]);
	}

	memdelete(p);
	points.remove(p_id);
	last_free_id = p_id;
}

void AStar::connect_points(int p_id, int p_with_id, bool bidirectional) {
	ERR_FAIL_COND_MSG(p_id == p_with_id, vformat("Can't connect point with id: %d to itself.", p_id));

	Point *a;
	bool from_exists = points.lookup(p_id, a);
	ERR_FAIL_COND_MSG(!from_exists, vformat("Can't connect points. Point with id: %d doesn't exist.", p_id));

	Point *b;
	bool to_exists = points.lookup(p_with_id, b);
	ERR_FAIL_COND_MSG(!to_exists, vformat("Can't connect points. Point with id: %d doesn't exist.", p_with_id));

	a->neighbours.set(b->id, b);

	if (bidirectional) {
		b->neighbours.set(a->id, a);
	} else {
		b->unlinked_neighbours.set(a->id, a);
	}

	Segment s(p_id, p_with_id);
	if (bidirectional) {
		s.direction = Segment::BIDIRECTIONAL;
	}

	Set<Segment>::Element *element = segments.find(s);
	if (element != nullptr) {
		s.direction |= element->get().direction;
		if (s.direction == Segment::BIDIRECTIONAL) {
			// Both are neighbours of each other now
			a->unlinked_neighbours.remove(b->id);
			b->unlinked_neighbours.remove(a->id);
		}
		segments.erase(element);
	}

	segments.insert(s);
}

void AStar::disconnect_points(int p_id, int p_with_id, bool bidirectional) {
	Point *a;
	bool a_exists = points.lookup(p_id, a);
	ERR_FAIL_COND_MSG(!a_exists, vformat("Can't disconnect points. Point with id: %d doesn't exist.", p_id));

	Point *b;
	bool b_exists = points.lookup(p_with_id, b);
	ERR_FAIL_COND_MSG(!b_exists, vformat("Can't disconnect points. Point with id: %d doesn't exist.", p_with_id));

	Segment s(p_id, p_with_id);
	int remove_direction = bidirectional ? (int)Segment::BIDIRECTIONAL : s.direction;

	Set<Segment>::Element *element = segments.find(s);
	if (element != nullptr) {
		// s is the new segment
		// Erase the directions to be removed
		s.direction = (element->get().direction & ~remove_direction);

		a->neighbours.remove(b->id);
		if (bidirectional) {
			b->neighbours.remove(a->id);
			if (element->get().direction != Segment::BIDIRECTIONAL) {
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

		segments.erase(element);
		if (s.direction != Segment::NONE) {
			segments.insert(s);
		}
	}
}

bool AStar::has_point(int p_id) const {
	return points.has(p_id);
}

Array AStar::get_points() {
	Array point_list;

	for (OAHashMap<int, Point *>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		point_list.push_back(*(it.key));
	}

	return point_list;
}

PoolVector<int> AStar::get_point_connections(int p_id) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, PoolVector<int>(), vformat("Can't get point's connections. Point with id: %d doesn't exist.", p_id));

	PoolVector<int> point_list;

	for (OAHashMap<int, Point *>::Iterator it = p->neighbours.iter(); it.valid; it = p->neighbours.next_iter(it)) {
		point_list.push_back((*it.key));
	}

	return point_list;
}

bool AStar::are_points_connected(int p_id, int p_with_id, bool bidirectional) const {
	Segment s(p_id, p_with_id);
	const Set<Segment>::Element *element = segments.find(s);

	return element != nullptr &&
			(bidirectional || (element->get().direction & s.direction) == s.direction);
}

void AStar::clear() {
	last_free_id = 0;
	for (OAHashMap<int, Point *>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {
		memdelete(*(it.value));
	}
	segments.clear();
	points.clear();
}

int AStar::get_point_count() const {
	return points.get_num_elements();
}

int AStar::get_point_capacity() const {
	return points.get_capacity();
}

void AStar::reserve_space(int p_num_nodes) {
	ERR_FAIL_COND_MSG(p_num_nodes <= 0, vformat("New capacity must be greater than 0, new was: %d.", p_num_nodes));
	ERR_FAIL_COND_MSG((uint32_t)p_num_nodes < points.get_capacity(), vformat("New capacity must be greater than current capacity: %d, new was: %d.", points.get_capacity(), p_num_nodes));
	points.reserve(p_num_nodes);
}

int AStar::get_closest_point(const Vector3 &p_point, bool p_include_disabled, int relevant_layers) const {
	int closest_id = -1;
	real_t closest_dist = 1e20;

	for (OAHashMap<int, Point *>::Iterator it = points.iter(); it.valid; it = points.next_iter(it)) {

		//make sure parallel layers are supported
		// or if *relevant_layers is 0 then use all points
		bool supported = relevant_layers == 0 || (relevant_layers & (*it.value)->parallel_support_layers) > 0;

		if (!p_include_disabled && !(*it.value)->enabled || !supported) {
			continue; // Disabled points should not be considered.
		}

		// Keep the closest point's ID, and in case of multiple closest IDs,
		// the smallest one (makes it deterministic).
		real_t d = p_point.distance_squared_to((*it.value)->pos);
		int id = *(it.key);
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

Vector3 AStar::get_closest_position_in_segment(const Vector3 &p_point) const {
	real_t closest_dist = 1e20;
	Vector3 closest_point;

	for (const Set<Segment>::Element *E = segments.front(); E; E = E->next()) {
		Point *from_point = nullptr, *to_point = nullptr;
		points.lookup(E->get().u, from_point);
		points.lookup(E->get().v, to_point);

		if (!(from_point->enabled && to_point->enabled)) {
			continue;
		}

		Vector3 segment[2] = {
			from_point->pos,
			to_point->pos,
		};

		Vector3 p = Geometry::get_closest_point_to_segment(p_point, segment);
		real_t d = p_point.distance_squared_to(p);
		if (d < closest_dist) {
			closest_point = p;
			closest_dist = d;
		}
	}

	return closest_point;
}

bool AStar::_solve(Point *begin_point, Point *end_point, int relevant_layers) {
	pass++;

	//make sure parallel layers are supported
	// or if *relevant_layers is 0 then use all points
	bool supported = relevant_layers == 0 || (relevant_layers & end_point->parallel_support_layers) > 0;
	if (!end_point->enabled || !supported) {
		return false;
	}

	bool found_route = false;

	Vector<Point *> open_list;
	SortArray<Point *, SortPoints> sorter;

	begin_point->g_score = 0;
	begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);
	open_list.push_back(begin_point);

	while (!open_list.empty()) {
		Point *p = open_list[0]; // The currently processed point

		if (p == end_point) {
			found_route = true;
			break;
		}

		sorter.pop_heap(0, open_list.size(), open_list.ptrw()); // Remove the current point from the open list
		open_list.remove(open_list.size() - 1);
		p->closed_pass = pass; // Mark the point as closed

		//if the point is part of an empty, look through all of the edge points of said empty (as to skip over any points within the empty).
		OAHashMap<int, Point*> connections;

		PoolVector<Empty*> enabled_empties;
		

		int size = p->empties.size();
		PoolVector<Empty*>::Read r = p->empties.read();
		for (int i = 0; i < size; i++) {

			
			Empty* e = r[i];
			

			supported = relevant_layers == 0 || (relevant_layers & e->parallel_support_layers) > 0;
			//if the empty is enabled and the end point is not within the empty
			if (e->enabled && supported && !end_point->empties.has(e)) {
				enabled_empties.append(e);
				//can travel to any edge point
				for (OAHashMap<int, Point*>::Iterator it = e->edge_points.iter(); it.valid; it = e->edge_points.next_iter(it)) {
					int id = *it.key;
					Point* ep = *(it.value);
					ep->is_neighbour = false;
					//don't connect to the same point 
					if (id != p->id && (i == 0 || !connections.has(id))) {
						connections.set(id, ep);
					}

				}
			}

		}

		
		
		//add neighbours to connections
		for (OAHashMap<int, Point*>::Iterator it = p->neighbours.iter(); it.valid; it = p->neighbours.next_iter(it)) {
			int id = *it.key;
			Point* np = *(it.value);// The neighbour point
			np->is_neighbour = true;
			//don't need to check for duplicate point connections if no empties
			if (size == 0 || !connections.has(id)) {

				//don't add points within enabled empties since they're meant to be skipped over
				if (np->empties.size() > 0 && !np->on_empty_edge) {
					bool in_enabled_empty = false;
					PoolVector<Empty*>::Read r1 = np->empties.read();
					for (int i = 0; i < np->empties.size(); i++) {
						if (enabled_empties.has(r1[i])) {
							in_enabled_empty = true;
							break;
						}
					}
					if (!in_enabled_empty) {
						connections.set(id, np);
					}
				}
				else {
					connections.set(id, np);
				}
				

			}
		}
		
		


		for (OAHashMap<int, Point *>::Iterator it = connections.iter(); it.valid; it = connections.next_iter(it)) {
			Point *e = *(it.value); // The neighbour point

			//make sure parallel layers are supported
			// or if *relevant_layers is 0 then use all points
			supported = relevant_layers == 0 || (relevant_layers & e->parallel_support_layers) > 0;
			

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
			e->prev_point_connected = e->is_neighbour;

			e->g_score = tentative_g_score;
			e->f_score = e->g_score + _estimate_cost(e->id, end_point->id);

			if (new_point) { // The position of the new points is already known.
				sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptrw());
			} else {
				sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptrw());
			}
		}
	}

	return found_route;
}

real_t AStar::_estimate_cost(int p_from_id, int p_to_id) {
	if (get_script_instance() && get_script_instance()->has_method(SceneStringNames::get_singleton()->_estimate_cost)) {
		return get_script_instance()->call(SceneStringNames::get_singleton()->_estimate_cost, p_from_id, p_to_id);
	}

	Point *from_point;
	bool from_exists = points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_from_id));

	Point *to_point;
	bool to_exists = points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t AStar::_compute_cost(int p_from_id, int p_to_id) {
	if (get_script_instance() && get_script_instance()->has_method(SceneStringNames::get_singleton()->_compute_cost)) {
		return get_script_instance()->call(SceneStringNames::get_singleton()->_compute_cost, p_from_id, p_to_id);
	}

	Point *from_point;
	bool from_exists = points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_from_id));

	Point *to_point;
	bool to_exists = points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

PoolVector<Vector3> AStar::get_point_path(int p_from_id, int p_to_id, int relevant_layers) {
	Point *a;
	bool from_exists = points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, PoolVector<Vector3>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_from_id));

	Point *b;
	bool to_exists = points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, PoolVector<Vector3>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		PoolVector<Vector3> ret;
		ret.push_back(a->pos);
		return ret;
	}

	Point *begin_point = a;
	Point *end_point = b;

	ERR_FAIL_INDEX_V(relevant_layers, ((1 << 31) - 1), PoolVector<Vector3>());

	bool found_route = _solve(begin_point, end_point, relevant_layers);
	if (!found_route) {
		return PoolVector<Vector3>();
	}

	Point *p = end_point;
	int pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	PoolVector<Vector3> path;
	path.resize(pc);
	skipped_connections_of_last_path_array.empty();
	skipped_connections_of_last_path_array.resize(pc);

	{
		PoolVector<Vector3>::Write w = path.write();
		PoolVector<uint8_t>::Write w2 = skipped_connections_of_last_path_array.write();

		Point *p2 = end_point;
		int idx = pc - 1;
		while (p2 != begin_point) {
			w[idx] = p2->pos;
			w2[idx--] = (p2->prev_point_connected ? 1:0);
			p2 = p2->prev_point;

		}

		w[0] = p2->pos; // Assign first
		w2[0] = 1;
		
	}

	return path;
}

PoolVector<int> AStar::get_id_path(int p_from_id, int p_to_id, int relevant_layers) {
	Point *a;
	bool from_exists = points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, PoolVector<int>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_from_id));

	Point *b;
	bool to_exists = points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, PoolVector<int>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		PoolVector<int> ret;
		ret.push_back(a->id);
		return ret;
	}

	Point *begin_point = a;
	Point *end_point = b;

	ERR_FAIL_INDEX_V(relevant_layers, ((1 << 31)-1), PoolVector<int>());

	bool found_route = _solve(begin_point, end_point, relevant_layers);
	if (!found_route) {
		return PoolVector<int>();
	}

	Point *p = end_point;
	int pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	PoolVector<int> path;
	path.resize(pc);
	skipped_connections_of_last_path_array.empty();
	skipped_connections_of_last_path_array.resize(pc);

	{
		PoolVector<int>::Write w = path.write();
		PoolVector<uint8_t>::Write w2 = skipped_connections_of_last_path_array.write();


		p = end_point;
		int idx = pc - 1;
		while (p != begin_point) {
			w[idx] = p->id;
			w2[idx--] = (p->prev_point_connected ? 1:0);
			p = p->prev_point;
		}

		w[0] = p->id; // Assign first
		w2[0] = 1;
	}

	return path;
}

PoolVector<uint8_t> AStar::get_skipped_connections_of_last_path_array()
{
	return skipped_connections_of_last_path_array;
}

void AStar::set_point_disabled(int p_id, bool p_disabled) {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set if point is disabled. Point with id: %d doesn't exist.", p_id));

	p->enabled = !p_disabled;

	// if point is part of an empty, disable the empties it is a part of if it is disaled
	int size = p->empties.size();
	if (size > 0) {
		PoolVector<Empty*>::Read r = p->empties.read();
		for (int i = 0; i < size; i++) {


			Empty* e = r[i];
			
			

			if (p_disabled) {
				e->disabled_points.append(p_id);
			}
			else {
				int i = e->disabled_points.find(p_id);
				e->disabled_points.remove(i);

			}


			// only enabled when containing no disabled points
			e->enabled = e->weighted_points.size() == 0 && e->disabled_points.size() == 0;
		}
	}

}

bool AStar::is_point_disabled(int p_id) const {
	Point *p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get if point is disabled. Point with id: %d doesn't exist.", p_id));

	return !p->enabled;
}



void AStar::set_point_layer(int p_id, int layer_index, bool l_enabled)
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_MSG(!p_exists, vformat("Can't set point layer index. Point with id: %d doesn't exist.", p_id));


	ERR_FAIL_INDEX(layer_index, 31);

	uint32_t layers = p->parallel_support_layers;

	if (l_enabled) {
		p->parallel_support_layers = layers | (1 << layer_index);
	}
	else {
		p->parallel_support_layers = layers & (~(1 << layer_index));
	}


	

	//changes to layers results in the removal of empties point is attached to, since it is no longer certain what layers are supported ater the change without looping through all the empty's points:
	//first fetch all e_ids to be removed
	PoolVector<int> removal_arr;
	PoolVector<Empty*>::Read r = p->empties.read();
	for (int i = 0; i < p->empties.size(); i++) {
		int e_id = r[i]->id;
		removal_arr.append(e_id);

	}

	PoolVector<int>::Read r1 = removal_arr.read();
	for (int i = 0; i < removal_arr.size(); i++) {
		remove_empty(r1[i]);
	}

}

bool AStar::get_point_layer(int p_id,int layer_index) const
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get if point supports layer index. Point with id: %d doesn't exist.", p_id));

	uint32_t layers = p->parallel_support_layers;
	ERR_FAIL_INDEX_V(layer_index, 31, false);
	return (layers & (1 << layer_index)) > 0;
}

int AStar::get_point_layers_value(int p_id) const
{
	Point* p;
	bool p_exists = points.lookup(p_id, p);
	ERR_FAIL_COND_V_MSG(!p_exists, false, vformat("Can't get the point's layers value. Point with id: %d doesn't exist.", p_id));

	return p->parallel_support_layers;
	
}


void AStar::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_available_point_id"), &AStar::get_available_point_id);
	ClassDB::bind_method(D_METHOD("add_point", "id", "position", "weight_scale","point_layers"), &AStar::add_point, DEFVAL(1.0), DEFVAL(0));
	ClassDB::bind_method(D_METHOD("add_empty", "id", "pool_points", "pool_edge_points"), &AStar::add_empty);
	ClassDB::bind_method(D_METHOD("debug_empty", "id"), &AStar::debug_empty);
	ClassDB::bind_method(D_METHOD("get_point_empty_ids", "id"), &AStar::get_point_empty_ids);
	ClassDB::bind_method(D_METHOD("get_empties"), &AStar::get_empties);
	ClassDB::bind_method(D_METHOD("remove_empty", "id"), &AStar::remove_empty);

	ClassDB::bind_method(D_METHOD("append_as_bulk_array", "pool_points", "max_connections", "pool_connections"), &AStar::append_as_bulk_array);
	ClassDB::bind_method(D_METHOD("set_as_bulk_array", "pool_points", "max_connections","pool_connections"), &AStar::set_as_bulk_array);

	ClassDB::bind_method(D_METHOD("get_point_position", "id"), &AStar::get_point_position);
	ClassDB::bind_method(D_METHOD("set_point_position", "id", "position"), &AStar::set_point_position);
	ClassDB::bind_method(D_METHOD("get_point_weight_scale", "id"), &AStar::get_point_weight_scale);
	ClassDB::bind_method(D_METHOD("set_point_weight_scale", "id", "weight_scale"), &AStar::set_point_weight_scale);
	ClassDB::bind_method(D_METHOD("remove_point", "id"), &AStar::remove_point);
	ClassDB::bind_method(D_METHOD("has_point", "id"), &AStar::has_point);
	ClassDB::bind_method(D_METHOD("get_point_connections", "id"), &AStar::get_point_connections);
	ClassDB::bind_method(D_METHOD("get_points"), &AStar::get_points);

	ClassDB::bind_method(D_METHOD("set_point_disabled", "id", "disabled"), &AStar::set_point_disabled, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("is_point_disabled", "id"), &AStar::is_point_disabled);

	ClassDB::bind_method(D_METHOD("set_point_layer", "id","layer_index","enabled"), &AStar::set_point_layer, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("get_point_layer", "id", "layer_index"), &AStar::get_point_layer);
	ClassDB::bind_method(D_METHOD("get_point_layers_value", "id"), &AStar::get_point_layers_value);
	

	ClassDB::bind_method(D_METHOD("connect_points", "id", "to_id", "bidirectional"), &AStar::connect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("disconnect_points", "id", "to_id", "bidirectional"), &AStar::disconnect_points, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("are_points_connected", "id", "to_id", "bidirectional"), &AStar::are_points_connected, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("get_point_count"), &AStar::get_point_count);
	ClassDB::bind_method(D_METHOD("get_point_capacity"), &AStar::get_point_capacity);
	ClassDB::bind_method(D_METHOD("reserve_space", "num_nodes"), &AStar::reserve_space);
	ClassDB::bind_method(D_METHOD("clear"), &AStar::clear);

	ClassDB::bind_method(D_METHOD("get_closest_point", "to_position", "include_disabled","relevant_layers"), &AStar::get_closest_point, DEFVAL(false), DEFVAL(0));
	ClassDB::bind_method(D_METHOD("get_closest_position_in_segment", "to_position"), &AStar::get_closest_position_in_segment);

	ClassDB::bind_method(D_METHOD("get_point_path", "from_id", "to_id", "relevant_layers"), &AStar::get_point_path, DEFVAL(0));
	ClassDB::bind_method(D_METHOD("get_id_path", "from_id", "to_id", "relevant_layers"), &AStar::get_id_path, DEFVAL(0));
	
	ClassDB::bind_method(D_METHOD("get_skipped_connections_of_last_path_array"), &AStar::get_skipped_connections_of_last_path_array);

	BIND_VMETHOD(MethodInfo(Variant::REAL, "_estimate_cost", PropertyInfo(Variant::INT, "from_id"), PropertyInfo(Variant::INT, "to_id")));
	BIND_VMETHOD(MethodInfo(Variant::REAL, "_compute_cost", PropertyInfo(Variant::INT, "from_id"), PropertyInfo(Variant::INT, "to_id")));
	

}

AStar::AStar() {
	last_free_id = 0;
	pass = 1;
}

AStar::~AStar() {
	clear();
}

/////////////////////////////////////////////////////////////

int AStar2D::get_available_point_id() const {
	return astar.get_available_point_id();
}

void AStar2D::add_point(int p_id, const Vector2 &p_pos, real_t p_weight_scale) {
	astar.add_point(p_id, Vector3(p_pos.x, p_pos.y, 0), p_weight_scale);
}

Vector2 AStar2D::get_point_position(int p_id) const {
	Vector3 p = astar.get_point_position(p_id);
	return Vector2(p.x, p.y);
}

void AStar2D::set_point_position(int p_id, const Vector2 &p_pos) {
	astar.set_point_position(p_id, Vector3(p_pos.x, p_pos.y, 0));
}

real_t AStar2D::get_point_weight_scale(int p_id) const {
	return astar.get_point_weight_scale(p_id);
}

void AStar2D::set_point_weight_scale(int p_id, real_t p_weight_scale) {
	astar.set_point_weight_scale(p_id, p_weight_scale);
}

void AStar2D::remove_point(int p_id) {
	astar.remove_point(p_id);
}

bool AStar2D::has_point(int p_id) const {
	return astar.has_point(p_id);
}

PoolVector<int> AStar2D::get_point_connections(int p_id) {
	return astar.get_point_connections(p_id);
}

Array AStar2D::get_points() {
	return astar.get_points();
}

void AStar2D::set_point_disabled(int p_id, bool p_disabled) {
	astar.set_point_disabled(p_id, p_disabled);
}

bool AStar2D::is_point_disabled(int p_id) const {
	return astar.is_point_disabled(p_id);
}

void AStar2D::connect_points(int p_id, int p_with_id, bool p_bidirectional) {
	astar.connect_points(p_id, p_with_id, p_bidirectional);
}

void AStar2D::disconnect_points(int p_id, int p_with_id, bool p_bidirectional) {
	astar.disconnect_points(p_id, p_with_id, p_bidirectional);
}

bool AStar2D::are_points_connected(int p_id, int p_with_id, bool p_bidirectional) const {
	return astar.are_points_connected(p_id, p_with_id, p_bidirectional);
}

int AStar2D::get_point_count() const {
	return astar.get_point_count();
}

int AStar2D::get_point_capacity() const {
	return astar.get_point_capacity();
}

void AStar2D::clear() {
	astar.clear();
}

void AStar2D::reserve_space(int p_num_nodes) {
	astar.reserve_space(p_num_nodes);
}

int AStar2D::get_closest_point(const Vector2 &p_point, bool p_include_disabled) const {
	return astar.get_closest_point(Vector3(p_point.x, p_point.y, 0), p_include_disabled);
}

Vector2 AStar2D::get_closest_position_in_segment(const Vector2 &p_point) const {
	Vector3 p = astar.get_closest_position_in_segment(Vector3(p_point.x, p_point.y, 0));
	return Vector2(p.x, p.y);
}

real_t AStar2D::_estimate_cost(int p_from_id, int p_to_id) {
	if (get_script_instance() && get_script_instance()->has_method(SceneStringNames::get_singleton()->_estimate_cost)) {
		return get_script_instance()->call(SceneStringNames::get_singleton()->_estimate_cost, p_from_id, p_to_id);
	}

	AStar::Point *from_point;
	bool from_exists = astar.points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_from_id));

	AStar::Point *to_point;
	bool to_exists = astar.points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't estimate cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

real_t AStar2D::_compute_cost(int p_from_id, int p_to_id) {
	if (get_script_instance() && get_script_instance()->has_method(SceneStringNames::get_singleton()->_compute_cost)) {
		return get_script_instance()->call(SceneStringNames::get_singleton()->_compute_cost, p_from_id, p_to_id);
	}

	AStar::Point *from_point;
	bool from_exists = astar.points.lookup(p_from_id, from_point);
	ERR_FAIL_COND_V_MSG(!from_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_from_id));

	AStar::Point *to_point;
	bool to_exists = astar.points.lookup(p_to_id, to_point);
	ERR_FAIL_COND_V_MSG(!to_exists, 0, vformat("Can't compute cost. Point with id: %d doesn't exist.", p_to_id));

	return from_point->pos.distance_to(to_point->pos);
}

PoolVector<Vector2> AStar2D::get_point_path(int p_from_id, int p_to_id) {
	AStar::Point *a;
	bool from_exists = astar.points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, PoolVector<Vector2>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_from_id));

	AStar::Point *b;
	bool to_exists = astar.points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, PoolVector<Vector2>(), vformat("Can't get point path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		PoolVector<Vector2> ret;
		ret.push_back(Vector2(a->pos.x, a->pos.y));
		return ret;
	}

	AStar::Point *begin_point = a;
	AStar::Point *end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return PoolVector<Vector2>();
	}

	AStar::Point *p = end_point;
	int pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	PoolVector<Vector2> path;
	path.resize(pc);

	{
		PoolVector<Vector2>::Write w = path.write();

		AStar::Point *p2 = end_point;
		int idx = pc - 1;
		while (p2 != begin_point) {
			w[idx--] = Vector2(p2->pos.x, p2->pos.y);
			p2 = p2->prev_point;
		}

		w[0] = Vector2(p2->pos.x, p2->pos.y); // Assign first
	}

	return path;
}

PoolVector<int> AStar2D::get_id_path(int p_from_id, int p_to_id) {
	AStar::Point *a;
	bool from_exists = astar.points.lookup(p_from_id, a);
	ERR_FAIL_COND_V_MSG(!from_exists, PoolVector<int>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_from_id));

	AStar::Point *b;
	bool to_exists = astar.points.lookup(p_to_id, b);
	ERR_FAIL_COND_V_MSG(!to_exists, PoolVector<int>(), vformat("Can't get id path. Point with id: %d doesn't exist.", p_to_id));

	if (a == b) {
		PoolVector<int> ret;
		ret.push_back(a->id);
		return ret;
	}

	AStar::Point *begin_point = a;
	AStar::Point *end_point = b;

	bool found_route = _solve(begin_point, end_point);
	if (!found_route) {
		return PoolVector<int>();
	}

	AStar::Point *p = end_point;
	int pc = 1; // Begin point
	while (p != begin_point) {
		pc++;
		p = p->prev_point;
	}

	PoolVector<int> path;
	path.resize(pc);

	{
		PoolVector<int>::Write w = path.write();

		p = end_point;
		int idx = pc - 1;
		while (p != begin_point) {
			w[idx--] = p->id;
			p = p->prev_point;
		}

		w[0] = p->id; // Assign first
	}

	return path;
}

bool AStar2D::_solve(AStar::Point *begin_point, AStar::Point *end_point) {
	astar.pass++;

	if (!end_point->enabled) {
		return false;
	}

	bool found_route = false;

	Vector<AStar::Point *> open_list;
	SortArray<AStar::Point *, AStar::SortPoints> sorter;

	begin_point->g_score = 0;
	begin_point->f_score = _estimate_cost(begin_point->id, end_point->id);
	open_list.push_back(begin_point);

	while (!open_list.empty()) {
		AStar::Point *p = open_list[0]; // The currently processed point

		if (p == end_point) {
			found_route = true;
			break;
		}

		sorter.pop_heap(0, open_list.size(), open_list.ptrw()); // Remove the current point from the open list
		open_list.remove(open_list.size() - 1);
		p->closed_pass = astar.pass; // Mark the point as closed

		for (OAHashMap<int, AStar::Point *>::Iterator it = p->neighbours.iter(); it.valid; it = p->neighbours.next_iter(it)) {
			AStar::Point *e = *(it.value); // The neighbour point

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
				sorter.push_heap(0, open_list.size() - 1, 0, e, open_list.ptrw());
			} else {
				sorter.push_heap(0, open_list.find(e), 0, e, open_list.ptrw());
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
	ClassDB::bind_method(D_METHOD("get_points"), &AStar2D::get_points);

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

	BIND_VMETHOD(MethodInfo(Variant::REAL, "_estimate_cost", PropertyInfo(Variant::INT, "from_id"), PropertyInfo(Variant::INT, "to_id")));
	BIND_VMETHOD(MethodInfo(Variant::REAL, "_compute_cost", PropertyInfo(Variant::INT, "from_id"), PropertyInfo(Variant::INT, "to_id")));
}

AStar2D::AStar2D() {
}

AStar2D::~AStar2D() {
}
