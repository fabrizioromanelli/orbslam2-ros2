/**
 * @brief Extrasampler abstract class.
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

/**
 * @brief An extrasampler requires a specific numeric type used to represent
 *        samples and other floating-point data. Meant for public inheritance.
 */

#ifndef EXTRASAMPLER_HPP
#define EXTRASAMPLER_HPP

template <typename NumericType>
class Extrasampler
{
public:
    virtual NumericType get_sample(NumericType time) = 0;
    virtual void update_samples(NumericType new_time, NumericType new_sample) = 0;
    virtual void reset(void) = 0;

protected:
    unsigned int samples_rcvd_;
};

#endif
