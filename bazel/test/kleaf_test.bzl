# SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
# Copyright (C) 2026 The Android Open Source Project

"""Tests on Kleaf using ACK / GKI as a baseline."""

load("@bazel_skylib//rules:build_test.bzl", "build_test")

_ALLOW_DDK_UNSAFE_HEADERS_SETTING = "//build/kernel/kleaf:allow_ddk_unsafe_headers"

def _ddk_unsafe_headers_transition_impl(_settings, _attr):
    return {_ALLOW_DDK_UNSAFE_HEADERS_SETTING: True}

_ddk_unsafe_headers_transition = transition(
    implementation = _ddk_unsafe_headers_transition_impl,
    inputs = [],
    outputs = [_ALLOW_DDK_UNSAFE_HEADERS_SETTING],
)

# This is a simple wrapper to trigger the transition to add unsafe headers.
def _ddk_headers_wrapper_impl(ctx):
    return ctx.attr.target[DefaultInfo]

_ddk_headers_wrapper = rule(
    implementation = _ddk_headers_wrapper_impl,
    # test = True,
    attrs = {
        "target": attr.label(
            cfg = "exec",
        ),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
    cfg = _ddk_unsafe_headers_transition,
)

def ddk_headers_build_test(
        name,
        **kwargs):
    """Define a test to check DDK headers build correctly.

    Args:
        name: Name of the test
        **kwargs: additional kwargs common to all rules.
    """

    _ddk_headers_wrapper(
        name = name + "_ddk_headers_wrapped",
        target = "//common:all_headers",
    )

    build_test(
        name = name,
        targets = [
            name + "_ddk_headers_wrapped",
        ],
        **kwargs
    )
