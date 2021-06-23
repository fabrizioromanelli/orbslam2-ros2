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

#include <math.h>

/**
 * @brief Creates a linear extrasampler.
 *
 * @param init_time Initial value of the last sample acquisition time.
 * @param init_sample First available sample.
 */
template <typename NumericType, unsigned int Samples>
LinearExtrasampler<NumericType, Samples>::LinearExtrasampler()
{
    // Set private members.
    this->reset();
}

/**
 * @brief Computes a new predicted sample extrapolated from the stored one.
 *
 * @param time Time at which to compute the new sample.
 * @return New extrapolated sample.
 */
template <typename NumericType, unsigned int Samples>
NumericType LinearExtrasampler<NumericType, Samples>::get_sample(NumericType time)
{
    if (this->samples_rcvd_ == Samples)
    {
        // Simply apply a linear approximation.
        return a_ * time + b_;
    }
    return NAN;
}

/**
 * @brief Updates the last stored true sample.
 *
 * @param new_time Acquisition time.
 * @param new_sample Newly acquired sample.
 */
template <typename NumericType, unsigned int Samples>
void LinearExtrasampler<NumericType, Samples>::update_samples(NumericType new_time, NumericType new_sample)
{
    // Store latest sample.
    times_buffer_[new_sample_index_] = new_time;
    samples_buffer_[new_sample_index_] = new_sample;
    new_sample_index_ = (new_sample_index_ + 1) % Samples;
    if (this->samples_rcvd_ == Samples)
    {
        // Update linear regressor coefficients.
        NumericType sigmaxy = sum_and_multiply(times_buffer_, samples_buffer_);
        NumericType sigmaxx = sum_and_multiply(times_buffer_, times_buffer_);
        NumericType sigmax = sum(times_buffer_);
        NumericType sigmay = sum(samples_buffer_);

        b_ = (NumericType(Samples) * sigmaxy - sigmax * sigmay) / (NumericType(Samples) * sigmaxx - sigmax * sigmax);
        a_ = (sigmay * sigmaxx - sigmax * sigmaxy) / (NumericType(Samples) * sigmaxx - sigmax * sigmax);
    }
    else
    {
        this->samples_rcvd_++;
    }
}

/**
 * @brief Sums over an array of values.
 *
 * @param var Pointer to the array to sum over.
 * @return Sum.
 */
template <typename NumericType, unsigned int Samples>
NumericType LinearExtrasampler<NumericType, Samples>::sum(NumericType *var)
{
    NumericType accumulator(0);
    for (unsigned int i = 0; i < Samples; i++)
        accumulator += var[i];
    return accumulator;
}

/**
 * @brief Computes the dot product of two arrays of values.
 *
 * @param var1 First array pointer.
 * @param var2 Second array pointer.
 * @return Dot product result.
 */
template <typename NumericType, unsigned int Samples>
NumericType LinearExtrasampler<NumericType, Samples>::sum_and_multiply(NumericType *var1, NumericType *var2)
{
    NumericType accumulator(0);
    for (unsigned int i = 0; i < Samples; i++)
        accumulator += (var1[i] * var2[i]);
    return accumulator;
}

/**
 * @brief Resets the internal state of the extrasampler.
 */
template <typename NumericType, unsigned int Samples>
void LinearExtrasampler<NumericType, Samples>::reset(void)
{
    a_ = NumericType(0);
    b_ = NumericType(0);
    for (unsigned int i = 0; i < Samples; i++)
    {
        samples_buffer_[i] = NumericType(0);
        times_buffer_[i] = NumericType(0);
    }
    new_sample_index_ = 0;
    this->samples_rcvd_ = 0;
}

#endif
