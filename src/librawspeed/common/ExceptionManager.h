/*
    RawSpeed - RAW file decoder.

    Copyright (C) 2023 Roman Lebedev

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#pragma once

namespace rawspeed {

template <typename Derived, typename Thrower> class ExceptionManagerBase {
protected:
  static void emitException() { Thrower()(); }

public:
  void exceptionalSituationEncountered();
};

template <typename Thrower>
class ImmediateExceptionThrower
    : public ExceptionManagerBase<ImmediateExceptionThrower<Thrower>, Thrower> {
  using Base =
      ExceptionManagerBase<ImmediateExceptionThrower<Thrower>, Thrower>;

public:
  void exceptionalSituationEncountered() const { Base::emitException(); }
};

template <typename Thrower>
class DeferredExceptionScope
    : public ExceptionManagerBase<DeferredExceptionScope<Thrower>, Thrower> {
  using Base = ExceptionManagerBase<DeferredExceptionScope<Thrower>, Thrower>;

  bool haveDeferredException = false;

public:
  void exceptionalSituationEncountered() { haveDeferredException = true; }

  ~DeferredExceptionScope() {
    if (haveDeferredException)
      Base::emitException();
  }
};

} // namespace rawspeed
