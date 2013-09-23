PROJECT = ebuggy

DEPS = chronos

#dep_erlang_ale = https://github.com/esl/erlang_ale master
dep_chronos = https://github.com/lehoff/chronos master

include erlang.mk

REBAR_DEPS_DIR=${DEPS_DIR}
