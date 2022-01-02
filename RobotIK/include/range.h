#ifndef __RANGE_H
#define __RANGE_H

#include <limits>
#include <algorithm>
#include <iostream>
#include <cmath>

#include <vector>

#ifndef WARN_IF_UNUSED
#define WARN_IF_UNUSED __attribute__((warn_unused_result))
#endif

template<class T>
T constrain(T _min, T _value, T _max) {
    return (_value < _min)
        ? _min
        : (_value > _max)
            ? _max
            : _value;
}


template<class T, class TLimits=std::numeric_limits<T> >
class range_t
{
public:
  T begin;
  T end;

  typedef TLimits Limits;

  inline static range_t max() { return range_t(TLimits::min(), TLimits::max()); }

  //typedef std::vector<range_t> list;
  class list : public std::vector<range_t>
  {
  public:
	using std::vector<range_t>::size;
	using typename std::vector<range_t>::iterator;
	using std::vector<range_t>::begin;
	using std::vector<range_t>::end;
	using std::vector<range_t>::erase;

	static list one(range_t r) {
		list l;
		l.insert(l.end(), r);
		return l;
	}

	void normalize() 
	{
		// make sure our range list is ordered, and non-intersecting
		if(std::vector<range_t>::size() < 2) return;	// 0 or 1 list is by definition normalized
		// sort our list of ranges
		sort(begin(), end());
		typename list::iterator i=begin(), j=begin()+1, _j=end();
		while(j!=_j) {
			if(j->begin < i->end) {	// j.begin can equal i.end because i.end is not inclusive
				// j's range is within i's range
				i->end = std::max(i->end, j->end);
				erase(j); j = i+1; _j=end();
			} else
				i++,j++;
		}
	}

	T totalSpan() const {
		T acc=0;
		for(typename list::const_iterator i=begin(), _i=end(); i!=_i; i++)
			acc += i->span();
		return acc;
	}

	list operator+(const list& rhs)
	{
		// create the output list, add ranges from both lists, then sort, then iterate and combine intersecting ranges
		list out;
		out.insert(out.end(), begin(), end());
		out.insert(out.end(), rhs.begin(), rhs.end());
		sort(out.begin(), out.end());
		if(out.size()<2) return out;

		// now we merge any intersecting ranges, event when more than 2 ranges intersect, this will still merge all of them
		typename range_t<T>::list::iterator i=out.begin(), j=out.begin()+1, _j=out.end(); 
		while (j!=_j) {
			if(i->intersects(*j)) {
				*i = i->merge(*j);
				out.erase(j);
				j = i+1;
				_j = out.end();
			} else
				i++,j++;
		}
		return out;
	}

	list operator-(const list& rhs)
	{
		// add rows from the second list to the first, merging any ranges that overlap
		// copy both lists and sort them
		list out;
		for(typename list::const_iterator l=begin(), _l=end(); l!=_l; l++) {
			range_t<T> current( *l );
			for(typename list::const_iterator r=rhs.begin(), _r=rhs.end(); r!=_r; r++) {
				if(r->end <= current.begin || r->begin >= current.end) // check for non-intersection
					continue;
				if(r->isInsideOf(current)) {
					// r splits l
					out.insert(out.end(), range_t(current.begin, r->begin));
					current.begin = r->end;
				} else
					current = current - *r; // simple intersect
				if(current.zero())
					break;
			}

			// there are no more intersections of 'l', we can
			// now add 'current', which is our original or adjusted 'l', into the out list
			if(current.nonzero())
				out.insert(out.end(), current);
		}
		return out;
	}

  };

  inline range_t() : begin( TLimits::min() ), end( TLimits::max() ) {}
  inline range_t(T b, T e) : begin(b), end(e) {}

  //template<class Q>
  //inline range_t(const Q& has_begin_end) : begin(has_begin_end.begin), end(has_begin_end.end) {}


  inline void clear() { begin=(T)0; end=TLimits::max(); }

  inline T span() const { return end-begin; }

  inline bool nonzero() const { return end>begin; }
  inline bool zero() const { return end<=begin; }

  inline range_t reverse() const  WARN_IF_UNUSED
    { return range_t(end, begin); }

  inline bool operator<(const range_t& rhs) const { return begin < rhs.begin; }

  inline range_t snap(T stride) const WARN_IF_UNUSED {
	range_t s; 
	s.begin = floor((double)begin/stride)*stride; 
    if(s.end > TLimits::max()-stride)
        s.end = floor((double)(end/stride))*stride;
    else
	    s.end = floor((double)((end+stride-1)/stride))*stride; 
	return s; 
  } 

  bool intersects(const range_t& rhs, range_t* intersection=NULL) const
  {
	if(end<=rhs.begin || begin>=rhs.end)
		return false;
	else if(intersection) {
		intersection->begin = std::max(begin, rhs.begin);
		intersection->end = std::min(end, rhs.end);
	}
	return true;
  }
  inline range_t intersection(const range_t& rhs) const  WARN_IF_UNUSED
    { range_t v; return intersects(rhs, &v) ? v : range_t(0,0); }

  inline range_t operator|(const range_t& rhs) const  WARN_IF_UNUSED
    { return intersection(rhs); }

  bool contains(range_t rhs) const
  { 
	// this range must fully contain the 'rhs' range
	return begin <= rhs.begin && end >= rhs.end;
  }
  inline bool contains(const T& val) const { return val>=begin && val<end; }

  // constrain a value to the limits of this range
  inline T constrain(const T& value) const  WARN_IF_UNUSED
    { return (value<begin) ? begin : ((value>end) ? end : value); }

  // returns true if this range is inside of the given range and doesnt lay on an edge
  inline bool isInsideOf(const range_t& val) const { return val.contains(*this) && begin!=val.begin && end!=val.end; }

  // return the minimal range that contains both this range and the given one
  inline range_t merge(const range_t& rhs) const WARN_IF_UNUSED
    { return {std::min(begin, rhs.begin), std::max(end, rhs.end)}; }

  // add an offset to the range
  inline T offset(T amount) const  WARN_IF_UNUSED
    { return { begin + amount, end + amount}; }

  // remove a portion from this range as given by the supplied range
  void subtract(const range_t& intersecting_range) {
	if(!intersects(intersecting_range))
		return;
	if(begin < intersecting_range.begin)
		end = intersecting_range.begin; // trim right
	else
		begin = intersecting_range.end; // trim left
  }

#if 0
#define POST_UNARY_OPERATOR(OP)  inline range_t operator OP() const { range r(*this); begin##OP; end##OP; return r }
#define PRE_UNARY_OPERATOR(OP)  inline range_t& operator OP() { begin##OP; end##OP; return *this; }
#define BINARY_OPERATOR(OP)  inline range_t operator OP(const range_t& rhs) const { return range_t( begin OP rhs.begin, end OP rhs.end); }
  POST_UNARY_OPERATOR(++)
  PRE_UNARY_OPERATOR(++)
  POST_UNARY_OPERATOR(--)
  PRE_UNARY_OPERATOR(--)
#undef POST_UNARY_OPERATOR
#undef PRE_UNARY_OPERATOR
#endif

  inline bool operator==(const range_t<T>& rhs) const { return begin==rhs.begin && end==rhs.end; }
  inline bool operator!=(const range_t<T>& rhs) const { return begin!=rhs.begin || end!=rhs.end; }

  inline range_t operator+(T val) const { return range_t(begin+val, end+val); }
  inline range_t operator-(T val) const { return range_t(begin-val, end-val); }
  inline range_t operator*(T val) const { return range_t(begin*val, end*val); }
  inline range_t operator/(T val) const { return range_t(begin/val, end/val); }

  inline range_t& operator+=(T val) { begin+=val; end+=val; return *this; }
  inline range_t& operator-=(T val) { begin-=val; end-=val; return *this; }
  inline range_t& operator*=(T val) { begin*=val; end*=val; return *this; }
  inline range_t& operator/=(T val) { begin/=val; end/=val; return *this; }

  range_t operator-(const range_t& rhs) const {
    range_t r(*this);
    if(end<=rhs.begin || begin>=rhs.end) // total miss
        return r;
    else if(rhs.end >= end && rhs.begin > begin) // right side crop
      r.end = rhs.begin;
    else if(rhs.begin <= begin && rhs.end > begin && rhs.end < end) // left side crop
      r.begin = rhs.end;
    else if(begin==rhs.begin && end==rhs.end)
      r.end = r.begin;	// equality case returns a null range, but keeps the left time
    else
      assert(false);	// rhs range splits this range or consumes it, operation would cause a loss of data
  }

  // operators working with lists
  list operator-(const list& _lst) const
  {
	// given the input list, return a list that covers 'this' range, but has the given list of ranges removed
	// copy this range, and copy the list of ranges so we can sort them
	list lst(_lst), out;
	sort(lst.begin(), lst.end());
	
	T lend=begin;
	for(typename list::const_iterator l=lst.begin(), _l=lst.end(); l!=_l; l++) {
		if(lend < l->begin)
			out.insert(out.end(), range_t(lend, l->begin));
		lend = l->end;
	}
	if(lend < end)
		out.insert(out.end(), range_t(lend, end));
	return out;
  }
};

#if 1

#else // the old way
template<class T>
typename range_t<T>::list operator+(const typename range_t<T>::list& lhs, const typename range_t<T>::list& rhs)
{
	// add rows from the second list to the first, merging any ranges that overlap
	// copy both lists and sort them
	range_t<T>::list left_sorted(lhs), right_sorted(rhs), out;
	sort(left_sorted.begin(), left_sorted.end());
	sort(right_sorted.begin(), right_sorted.end());

	// now iterate both lists at the same time, add to the output list the lesser range, or if the ranges intersect, then merge and add
	range_t<T>::list::const_iterator li = left_sorted.begin(), ri=right_sorted.end();
	while( li!=left_sorted.end() && ri!=right_sorted.end())
	{
		if(li->intersects(*ri)) {
			out.insert(out.end(), li->merge(*ri));
			li++; ri++;
		} if(*li < *ri) {
			out.insert(out.end(), *li)
			li++;
		} else {
			out.insert(out.end(), *ri);
			ri++;
		}
	}
	if(li!=left_sorted.end())
		out.insert(out.end(), li, left_sorted.end());
	if(ri!=right_sorted.end())
		out.insert(out.end(), ri, right_sorted.end());
	return out;
}
#endif

template<class T>
typename range_t<T>::list operator|(const typename range_t<T>::list& lhs, const typename range_t<T>::list& rhs)
{
	// create the output list, add ranges from both lists, then sort, then iterate and combine intersecting ranges
	typename range_t<T>::list out;
	out.insert(out.end(), lhs.begin(), lhs.end());
	out.insert(out.end(), rhs.begin(), rhs.end());
	sort(out.begin(), out.end());
	if(out.size()<2) return out;

	// now we merge any intersecting ranges, event when more than 2 ranges intersect, this will still merge all of them
	range_t<T> intersect;
	typename range_t<T>::list::iterator i=out.begin(), j=out.begin()+1, _j=out.end(); 
	while (j!=_j) {
		if(i->intersects(*j, &intersect)) {
			*i = intersect;
			out.erase(j);
			j = i+1;
			_j = out.end();
		} else
			i++,j++;
	}
	return out;
}

template<class T>
std::ostream & operator << (std::ostream &out, const range_t<T> &range)
{
    out << '[' << range.begin << ", " << range.end << ']';
    return out;
}

// standard int range types
typedef range_t<int64_t> range64_t;
typedef range_t<int32_t> range32_t;
typedef range_t<int16_t> range16_t;

typedef range_t<uint64_t> urange64_t;
typedef range_t<uint32_t> urange32_t;
typedef range_t<uint16_t> urange16_t;

// standard floating point range types
typedef range_t<double> drange_t;
typedef range_t<float> frange_t;

#if 0 // now provided in range_t<>::list
template<class T>
typename range_t<T>::list operator-(const typename range_t<T>::list& lhs, const typename range_t<T>::list& rhs)
{
	// add rows from the second list to the first, merging any ranges that overlap
	// copy both lists and sort them
	range_t<T>::list out;
	for(range_t<T>::list::const_iterator l=lhs.begin(), _l=lhs.end(); l!=_l; l++) {
		for(range_t<T>::list::const_iterator r=rhs.begin(), _r=rhs.end(); r!=_r; r++) {
			if(r->end <= l.begin || r.begin >= l.end)
				continue;
			out.insert(out.end(), *l - *r);
		}
	}
	return out;
}
#endif



#endif // __RANGE_H
