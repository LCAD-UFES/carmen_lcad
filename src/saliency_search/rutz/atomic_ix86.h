/*!@file rutz/atomic_ix86.h Inline x86 assembly functions imported
         from Linux kernel headers circa version 2.4.18-41mdk. */

// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic_ix86.h $
// $Id: atomic_ix86.h 7063 2006-08-29 18:26:55Z rjpeters $

#ifndef RUTZ_ATOMIC_IX86_H_UTC20050728203342_DEFINED
#define RUTZ_ATOMIC_IX86_H_UTC20050728203342_DEFINED

namespace rutz
{

/*
 * Atomic operations that C can't guarantee us.  Useful for
 * resource counting etc..
 */

#define LOCK "lock ; "

/*
 * Make sure gcc doesn't try to be clever and move things around
 * on us. We need to use _exactly_ the address the user gave us,
 * not some alias that contains the same information.
 */
typedef struct { volatile int counter; } ix86_atomic_int_t;

/**
 * ix86_atomic_read - read atomic variable
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically reads the value of @v.  Note that the guaranteed
 * useful range of an ix86_atomic_int_t is only 24 bits.
 */
#define ix86_atomic_read(v)          ((v)->counter)

/**
 * ix86_atomic_set - set atomic variable
 * @v: pointer of type ix86_atomic_int_t
 * @i: required value
 *
 * Atomically sets the value of @v to @i.  Note that the guaranteed
 * useful range of an ix86_atomic_int_t is only 24 bits.
 */
#define ix86_atomic_set(v,i)         (((v)->counter) = (i))

/**
 * ix86_atomic_add - add integer to atomic variable
 * @i: integer value to add
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically adds @i to @v.
 */
static __inline__ void ix86_atomic_add(int i, ix86_atomic_int_t *v)
{
        __asm__ __volatile__(
                LOCK "addl %1,%0"
                :"=m" (v->counter)
                :"ir" (i), "m" (v->counter));
}

/**
 * ix86_atomic_sub - subtract the atomic variable
 * @i: integer value to subtract
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically subtracts @i from @v.
 */
static __inline__ void ix86_atomic_sub(int i, ix86_atomic_int_t *v)
{
        __asm__ __volatile__(
                LOCK "subl %1,%0"
                :"=m" (v->counter)
                :"ir" (i), "m" (v->counter));
}

/**
 * ix86_atomic_sub_and_test - subtract value from variable and test result
 * @i: integer value to subtract
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically subtracts @i from @v and returns
 * true if the result is zero, or false for all
 * other cases.
 */
static __inline__ int ix86_atomic_sub_and_test(int i, ix86_atomic_int_t *v)
{
        unsigned char c;

        __asm__ __volatile__(
                LOCK "subl %2,%0; sete %1"
                :"=m" (v->counter), "=qm" (c)
                :"ir" (i), "m" (v->counter) : "memory");
        return c;
}

/**
 * ix86_atomic_inc - increment atomic variable
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically increments @v by 1.  Note that the guaranteed
 * useful range of an ix86_atomic_int_t is only 24 bits.
 */
static __inline__ void ix86_atomic_inc(ix86_atomic_int_t *v)
{
        __asm__ __volatile__(
                LOCK "incl %0"
                :"=m" (v->counter)
                :"m" (v->counter));
}

/**
 * ix86_atomic_dec - decrement atomic variable
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically decrements @v by 1.
 */
static __inline__ void ix86_atomic_dec(ix86_atomic_int_t *v)
{
        __asm__ __volatile__(
                LOCK "decl %0"
                :"=m" (v->counter)
                :"m" (v->counter));
}

/**
 * ix86_atomic_dec_and_test - decrement and test
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically decrements @v by 1 and
 * returns true if the result is 0, or false for all other
 * cases.  Note that the guaranteed
 * useful range of an ix86_atomic_int_t is only 24 bits.
 */
static __inline__ int ix86_atomic_dec_and_test(ix86_atomic_int_t *v)
{
        unsigned char c;

        __asm__ __volatile__(
                LOCK "decl %0; sete %1"
                :"=m" (v->counter), "=qm" (c)
                :"m" (v->counter) : "memory");
        return c != 0;
}

/**
 * ix86_atomic_inc_and_test - increment and test
 * @v: pointer of type ix86_atomic_int_t
 *
 * Atomically increments @v by 1
 * and returns true if the result is zero, or false for all
 * other cases.
 */
static __inline__ int ix86_atomic_inc_and_test(ix86_atomic_int_t *v)
{
        unsigned char c;

        __asm__ __volatile__(
                LOCK "incl %0; sete %1"
                :"=m" (v->counter), "=qm" (c)
                :"m" (v->counter) : "memory");
        return c != 0;
}

/**
 * ix86_atomic_add_return - add and return
 * @v: pointer of type ix86_atomic_int_t
 * @i: integer value to add
 *
 * Atomically adds @i to @v and returns @i + @v
 */
static __inline__ int ix86_atomic_add_return(int i, ix86_atomic_int_t *v)
{
        /* Modern 486+ processor */
        int __i = i;
        __asm__ __volatile__(
                LOCK "xaddl %0, %1;"
                :"=r"(i)
                :"m"(v->counter), "0"(i));
        return i + __i;
}

static __inline__ int ix86_atomic_sub_return(int i, ix86_atomic_int_t *v)
{
        return ix86_atomic_add_return(-i,v);
}

/// Atomic integer class for ix86 CPUs.
class ix86_atomic_int
{
private:
  ix86_atomic_int_t x;

  ix86_atomic_int(const ix86_atomic_int&); // not implemented
  ix86_atomic_int& operator=(const ix86_atomic_int&); // not implemented

public:
  //! Construct with an initial value of 0.
  ix86_atomic_int() { this->atomic_set(0); }

  //! Get the maximum representable value
  static int max_value() { return ((1 << 24) - 1); }

  //! Get the current value.
  int atomic_get() const
  { return ix86_atomic_read(&x); }

  //! Set value to the given value \a v.
  void atomic_set(int v)
  { ix86_atomic_set(&x, v); }

  //! Add \a v to the value.
  void atomic_add(int i)
  { ix86_atomic_add(i, &x); }

  //! Subtract \a v from the value.
  void atomic_sub(int i)
  { ix86_atomic_sub(i, &x); }

  //! Subtract \a v from the value; return true if the new value is zero.
  bool atomic_sub_test_zero(int i)
  { return bool(ix86_atomic_sub_and_test(i, &x)); }

  //! Increment the value by one.
  void atomic_incr()
  { ix86_atomic_inc(&x); }

  //! Decrement the value by one.
  void atomic_decr()
  { ix86_atomic_dec(&x); }

  //! Decrement the value by one; return true if the new value is zero.
  bool atomic_decr_test_zero()
  { return bool(ix86_atomic_dec_and_test(&x)); }

  //! Increment the value by one; return true if the new value is zero.
  bool atomic_incr_test_zero()
  { return bool(ix86_atomic_inc_and_test(&x)); }

  //! Add \a v to the value and return the new value.
  int atomic_add_return(int i)
  { return ix86_atomic_add_return(i, &x); }

  //! Subtract \a v from the value and return the new value.
  int atomic_sub_return(int i)
  { return ix86_atomic_sub_return(i, &x); }

  //! Increment the value by one and return the new value.
  int atomic_incr_return()
  { return ix86_atomic_add_return(1, &x); }

  //! Decrement the value by one and return the new value.
  int atomic_decr_return()
  { return ix86_atomic_add_return(-1, &x); }
};

} // end namespace rutz

#endif // !RUTZ_ATOMIC_IX86_H_UTC20050728203342DEFINED
