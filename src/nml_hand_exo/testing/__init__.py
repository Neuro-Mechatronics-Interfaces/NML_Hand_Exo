"""Test doubles for exercising host workflows without physical hardware."""

from .fake_openrb import FakeOpenRBComm, ReplyFault

__all__ = ["FakeOpenRBComm", "ReplyFault"]
