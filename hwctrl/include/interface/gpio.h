#pragma once

#include <string>

class Gpio {
 public:
  enum class State : uint8_t { Reset = 0, Set };

  enum class Direction : uint8_t { Input = 0, Output };

 public:
  explicit Gpio(uint32_t num, Direction dir = Direction::Input,
                State state = State::Reset);
  explicit Gpio(std::string, Direction dir = Direction::Input,
                State state = State::Reset);
  ~Gpio() = default;

  Gpio(Gpio const&) = delete;
  void operator=(Gpio const&) = delete;

  void set_direction(Direction) const;
  Direction get_direction() const;

  inline void in() { set_direction(Direction::Input); }
  inline void out() { set_direction(Direction::Output); }

  State read_state() const;
  void set_state(State) const;
    
  inline bool is_set() const { return read_state() == State::Set; }
  inline bool is_reset() const { return !is_set(); }

  inline void set() { set_state(State::Set); }
  inline void reset() { set_state(State::Reset); }

  inline std::string get_path() const { return m_path; }

 private:
  std::string m_path;
  std::string m_value_path;
  std::string m_dir_path;
};
