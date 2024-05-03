#pragma once

#include <esp_types.h>
#include <utility>

#define INL __attribute__((always_inline))

/**
 * @brief Uses Xtensa's zero-overhead loop to execute a given operation a number of times.
 * This function does \e not save/restore the LOOP registers, so if required these need to be 
 * saved&restored explicitly around the function call.
 * @note This may fail to assemble when compiled with \c -Og or less
 * 
 * @tparam F Type of the functor to execute
 * @tparam Args Argument types of the functor
 * @param cnt Number of iterations
 * @param f The functor to invoke
 * @param args Arguments to pass to the functor
 */
template<typename F, typename...Args>
static inline void INL rpt(const uint32_t cnt, const F& f, Args&&...args) {

    bgn:
        asm goto (
            "LOOPNEZ %[cnt], %l[end]"
            : /* no output*/
            : [cnt] "r" (cnt)
            : /* no clobbers */
            : end
        );

            f(std::forward<Args>(args)...);

 
    end:
        /* Tell the compiler that the above code might execute more than once.
           The begin label must be before the inital LOOP asm because otherwise
           gcc may decide to put one-off setup code between the LOOP asm and the
           begin label, i.e. inside the loop.
        */
        asm goto ("":::: bgn);    
        ;
}