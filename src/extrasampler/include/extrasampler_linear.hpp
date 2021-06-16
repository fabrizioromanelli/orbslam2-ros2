/**
 * @brief Linear extrapolation oversampler.
 *
 * @author Marco Passeri <mbass.pass@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date June 11, 2021
 */
/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
 */

#ifndef LINEAR_EXTRASAMPLER_HPP
#define LINEAR_EXTRASAMPLER_HPP

#include "extrasampler.hpp"

/**
 * @brief Extends extrasampler's base adding linear extrapolation methods.
 */
template <typename NumericType, unsigned int Samples>
class LinearExtrasampler : public Extrasampler<NumericType>
{
public:
    LinearExtrasampler();
    NumericType get_sample(NumericType time);
    void update_samples(NumericType new_time, NumericType new_sample);
    void reset(void);

private:
    NumericType a_;
    NumericType b_;
    NumericType samples_buffer_[Samples];
    NumericType times_buffer_[Samples];
    unsigned int new_sample_index_;
};

#include "extrasampler_linear.ipp"

#endif
