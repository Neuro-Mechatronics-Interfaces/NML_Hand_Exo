# phrase_handler.py
from __future__ import annotations
import math
import random
import time
from typing import Iterable, List, Optional, Callable

from nml_wtf_exo.utils.key_encodings import char_to_events

# A small pool of visible keys for “random typo” characters (US layout-ish)
_ERR_CHARS = (
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "`-=[]\\;',./"
)

def wpm_to_mean_ici_seconds(wpm: float) -> float:
    # 5 chars/word is the standard convention
    chars_per_min = max(1e-6, 5.0 * max(1.0, wpm))
    return 60.0 / chars_per_min

def _sample_error_char(target: str, rng: random.Random) -> str:
    """Pick a 'wrong' visible character not equal to the intended one."""
    t = target.lower()
    choices = [c for c in _ERR_CHARS if c != t]
    return rng.choice(choices) if choices else "x"

class PhraseHandler:
    """
    Loads phrases and 'types' them by calling an 'emit' function with JSON events.
    Timing between character events follows a log-normal distribution with
    mean determined by WPM and spread by coefficient of variation (cv).
    """

    phrases: List[str]
    emit: Callable[[dict], None]
    wpm: float = 60.0
    cv: float = 0.30
    rng: random.Random
    word_pause_mean: float = 0.0
    error_rate: float = 0.02
    error_burst_p: float = 0.70 
    current_error_keys: int = 0
    total_error_keys: int = 0

    def __init__(self,
                 phrases: Iterable[str],
                 emit: Callable[[dict], None],
                 wpm: float = 60.0,
                 cv: float = 0.30,
                 rng: Optional[random.Random] = None,
                 word_pause_mean: float = 0.0, 
                 error_rate: float = 0.02,
                 error_burst_p: float = 0.70):
        """
        phrases: lines of text to type (iterable)
        emit:    function taking a JSON-able dict (a keyboard message)
        wpm:     target typing speed (words per minute; 5 chars = 1 word)
        cv:      coefficient of variation for inter-character interval (>=0)
        rng:     optional random.Random instance
        word_pause_mean: extra mean pause (seconds) at spaces (0 = none)
        error_rate: probability an intended character triggers a typo episode
        error_burst_p: geometric stop prob for typo length (E[K] = 1/p)
        """
        self.phrases = list(phrases)
        self.emit = emit
        self.set_cv(cv, wpm)
        self.rng = rng or random.Random()
        self.word_pause_mean = max(0.0, word_pause_mean)
        self.error_rate = max(0.0, min(1.0, error_rate))
        self.error_burst_p = max(1e-6, min(1.0, error_burst_p))
        self.current_error_keys = 0
        self.total_error_keys = 0

    def set_wpm(self, wpm: float):
        self.wpm = wpm
        # Lognormal params from mean m and CV: sigma = sqrt(ln(1+cv^2)), mu = ln(m) - 0.5*sigma^2
        m = wpm_to_mean_ici_seconds(self.wpm)
        self._sigma = math.sqrt(math.log(1.0 + self.cv * self.cv)) if self.cv > 0 else 0.0
        self._mu = math.log(m) - 0.5 * (self._sigma ** 2)

    def set_cv(self, cv: float, wpm: float = None):
        self.cv = max(0.0, cv)
        if wpm is None:
            wpm = self.wpm
        self.set_wpm(wpm)

    def _sample_ici(self) -> float:
        if self._sigma == 0:
            return max(0.0, math.exp(self._mu))
        return self.rng.lognormvariate(self._mu, self._sigma)

    def _tap_backspace(self):
        self.emit({"type": "key", "key": "backspace", "action": "tap"})

    def _emit_char(self, ch: str):
        for ev in char_to_events(ch):
            self.emit(ev)

    def type_all(self, newline_between: bool = True):
        """
        Iterate through phrases and emit key events with timing.
        """
        for idx, line in enumerate(self.phrases):
            self.type_line(line.rstrip("\n"))
            if newline_between and idx < len(self.phrases) - 1:
                # end-of-line: press Enter
                for ev in char_to_events("\n"):
                    self.emit(ev)
                time.sleep(self._sample_ici())

    def type_line(self, text: str):
        i = 0
        pending_backspaces = 0  # how many errors we still need to remove

        while i < len(text) or pending_backspaces > 0:
            # If we owe backspaces, always pay them off first (as requested)
            if pending_backspaces > 0:
                self._tap_backspace()
                pending_backspaces -= 1
                time.sleep(self._sample_ici())
                continue

            # Finished text?
            if i >= len(text):
                break

            ch = text[i]

            # Decide whether to start a typo episode on THIS intended char
            if self.rng.random() < self.error_rate:
                # Draw typo length K ~ Geometric(p) on {1,2,...}
                # Equivalent: keep adding errors while U >= p; count the final success.
                self.current_error_keys = 1
                while self.rng.random() >= self.error_burst_p:
                    self.current_error_keys += 1

                # Emit K wrong keystrokes (with timing)
                for _ in range(self.current_error_keys):
                    wrong = _sample_error_char(ch, self.rng)
                    self._emit_char(wrong)
                    self.total_error_keys += 1
                    time.sleep(self._sample_ici())

                # Then schedule K backspaces (performed before any further typing)
                pending_backspaces = self.current_error_keys
                # Do NOT advance i yet; after backspaces, we'll try the intended char again
                continue

            # No typo: emit intended char
            self._emit_char(ch)
            i += 1

            # Timing
            dt = self._sample_ici()
            if ch == " " and self.word_pause_mean > 0:
                dt += self.rng.expovariate(1.0 / self.word_pause_mean)
            time.sleep(dt)