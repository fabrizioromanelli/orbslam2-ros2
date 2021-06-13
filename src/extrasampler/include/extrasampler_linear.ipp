/**
 * @brief Linear extrapolation oversampler source code.
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

#ifndef LINEAR_EXTRASAMPLER_IPP
#define LINEAR_EXTRASAMPLER_IPP

/**
 * @brief Creates a linear extrasampler.
 *
 * @param init_time Initial value of the last sample acquisition time.
 * @param init_sample First available sample.
 */
template <typename NumericType>
LinearExtrasampler<NumericType>::LinearExtrasampler(NumericType init_time, NumericType init_sample)
{
    // Set private members.
    prev_sample_time_ = init_time;
    prev_sample_ = init_sample;
}

/**
 * @brief Computes a new predicted sample extrapolated from the stored one.
 *
 * @param time Time at which to compute the new sample.
 * @return New extrapolated sample.
 */
template <typename NumericType>
NumericType LinearExtrasampler<NumericType>::get_sample(NumericType time)
{
    // Simply apply a linear approximation.
    return a_ * time + b_;
}

/**
 * @brief Updates the last stored true sample.
 *
 * @param new_time Acquisition time.
 * @param new_sample Newly acquired sample.
 */
template <typename NumericType>
void LinearExtrasampler<NumericType>::update_samples(NumericType new_time, NumericType new_sample)
{
    // Recompute linear approximation coefficients using new data.
    a_ = (new_sample - prev_sample_) / (new_time - prev_sample_time_);
    b_ = prev_sample_ - a_ * prev_sample_time_;
    // Store new values.
    prev_sample_ = new_sample;
    prev_sample_time_ = new_time;
}

/**
 * @brief Resets the internal state of the extrasampler.
 */
template <typename NumericType>
void LinearExtrasampler<NumericType>::reset(void)
{
    a_ = NumericType(0);
    b_ = NumericType(0);
    prev_sample_ = NumericType(0);
    prev_sample_time_ = NumericType(0);
}

#endif
