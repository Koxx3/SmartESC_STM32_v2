// Copyright 2015 Mark Owen
// http://www.quinapalus.com
// E-mail: qfp@quinapalus.com
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of version 2 of the GNU General Public License
// as published by the Free Software Foundation.
//
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this file.  If not, see <http://www.gnu.org/licenses/> or
// write to the Free Software Foundation, Inc., 51 Franklin Street,
// Fifth Floor, Boston, MA  02110-1301, USA.

#ifndef _QFPIO_H_
#define _QFPIO_H_

#ifdef __cplusplus
  extern "C" {
#endif

extern void qfp_float2str(float f,char*s,unsigned int fmt);
extern int qfp_str2float(float*f,char*p,char**endptr);

#ifdef __cplusplus
  } // extern "C"
#endif
#endif
