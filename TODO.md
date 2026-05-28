# CommandBasedTest — TODO

## Near-Term (Polish)

- [ ] Add private constructors to `CommandRunner` and `CommandRegisterer` to prevent accidental instantiation — both are pure static utility classes with implicit public constructors right now
- [ ] Warn via `Logger.recordOutput` when two actions within the same command share a name — duplicate names silently corrupt AdvantageKit log keys
- [ ] Log a warning when `withPriority` clamps a negative value to `0` so developers notice bad input rather than silently getting wrong behavior

## Medium-Term (Usability)

- [ ] **Test harness utility** — a lightweight class that drives a `CommandBase` through a sequence of cycles without real hardware, making unit testing possible
- [ ] **Cycle detection / transition guard** — log a warning if state changes on every consecutive cycle for more than N cycles on the same command, indicating a likely logic bug
- [ ] **Action builder DSL** — cleaner construction ergonomics that keep the same underlying model but reduce boilerplate when declaring actions with priority and subsystems

## Long-Term (Framework Maturity)

- [ ] **Transition hooks** — `onEnter(State)` / `onExit(State)` callbacks that fire automatically when state changes, so users don't have to manually track previous state inside actions
- [ ] **Action groups** — declare a set of actions as a unit for subsystem conflict resolution purposes

## Explicitly Out of Scope

- **Orthogonal states** — adds too much complexity for the target audience (FRC students) and breaks the core mental model of one state, sorted actions, threading. Transition hooks cover the main use case anyway.
