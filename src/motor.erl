-module(motor).

-behaviour(gen_server).

%% API
-export([start_link/0, stop_server/0]).

%% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
	 terminate/2, code_change/3]).

-export([forward/1, backward/1, rotate/2, stop/0]).

-define(SERVER, ?MODULE). 

-define(D90, 535). %% 90 degrees in milliseconds
-define(D180, 1222). %% 180 degrees in milliseconds

-record(state, 
	{theta,
	 requestor :: pid()}).

%%%===================================================================
%%% API
%%%===================================================================

%%--------------------------------------------------------------------
%% @doc
%% Starts the server
%%
%% @spec start_link() -> {ok, Pid} | ignore | {error, Error}
%% @end
%%--------------------------------------------------------------------
start_link() ->
    gen_server:start_link({local, ?SERVER}, ?MODULE, [], []).


forward(T) ->
    gen_server:call(?SERVER, {forward, T, self()}, infinity).

backward(T) ->
    gen_server:call(?SERVER, {backward, T, self()}, infinity).

rotate(T, left) ->
    gen_server:call(?SERVER, {rotate, T, left, self()}, infinity);
rotate(T, right) ->
    gen_server:call(?SERVER, {rotate, T, right, self()}, infinity).



stop() ->
    gen_server:call(?SERVER, stop).

stop_server() ->
    gen_server:cast(?SERVER, {cast, stop}).
%%%===================================================================
%%% gen_server callbacks
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Initializes the server
%%
%% @spec init(Args) -> {ok, State} |
%%                     {ok, State, Timeout} |
%%                     ignore |
%%                     {stop, Reason}
%% @end
%%--------------------------------------------------------------------
init([]) ->
    i2c_servo:init(),
    i2c_servo:setPWMFreq(60),
    {ok, _} = chronos:start_link(motor_ts),
    {ok, #position{x = 0, y = 0, theta = 0}}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling call messages
%%
%% @spec handle_call(Request, From, State) ->
%%                                   {reply, Reply, State} |
%%                                   {reply, Reply, State, Timeout} |
%%                                   {noreply, State} |
%%                                   {noreply, State, Timeout} |
%%                                   {stop, Reason, Reply, State} |
%%                                   {stop, Reason, State}
%% @end
%%--------------------------------------------------------------------

handle_call({forward, T}, _From, State) ->
    timer:sleep(10),
    chronos:start_timer(motor_timer, stop_motor, T, {motor, stop, []}),
    i2c_servo:setPWM(0, 0, 600),
    i2c_servo:setPWM(1, 0, 150),
    sharp:detect_obstacle(),
    {reply, Reply, State};


handle_call({backward, T}, _From, State) ->
    timer:sleep(10),
    i2c_servo:setPWM(0,0,150),
    i2c_servo:setPWM(1,0,600),
    timer:sleep(T),
    motor_stop(),
    Reply = ok,
    {reply, Reply, State};

handle_call({rotate, T, left}, _From, State) ->
    motor_rotate(T, left),
    Reply = ok,
    {reply, Reply, State};

handle_call({rotate, T, right}, _From, State) ->
    Reply = motor_rotate(T, right),
    {reply, Reply, State};

handle_call(stop, _From, #position{actions=[Next|Actions]}=State) ->
    motor_stop(),
    S1 = update_state(Next, State),
    case Actions of
	[] ->
	    Reply = ok,
	    {reply, Reply, S1#position{actions=[]}};
	[Action|Rest] ->
	    start_action(Action), 
	    {noreply, S1#position{actions=Rest}}
    end.

update_state({set, theta, V}, S) ->
    S#state{theta=V}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling cast messages
%%
%% @spec handle_cast(Msg, State) -> {noreply, State} |
%%                                  {noreply, State, Timeout} |
%%                                  {stop, Reason, State}
%% @end
%%--------------------------------------------------------------------
handle_cast({cast, stop}, State) ->
    {stop, normal, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling all non call/cast messages
%%
%% @spec handle_info(Info, State) -> {noreply, State} |
%%                                   {noreply, State, Timeout} |
%%                                   {stop, Reason, State}
%% @end
%%--------------------------------------------------------------------
handle_info({detect_obstacle, true}, State) ->
    {ok, Timer} = chronos:stop_timer(motor_timer, stop_timer),
    motor_stop(),
    {noreply, State}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% This function is called by a gen_server when it is about to
%% terminate. It should be the opposite of Module:init/1 and do any
%% necessary cleaning up. When it returns, the gen_server terminates
%% with Reason. The return value is ignored.
%%
%% @spec terminate(Reason, State) -> void()
%% @end
%%--------------------------------------------------------------------
terminate(_Reason, _State) ->
    ok.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Convert process state when code is changed
%%
%% @spec code_change(OldVsn, State, Extra) -> {ok, NewState}
%% @end
%%--------------------------------------------------------------------
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

%%%===================================================================
%%% Internal functions
%%%===================================================================

motor_forward(T) ->
    timer:sleep(10),
    chronos:start_timer(motor_timer, stop_motor, T, {motor, stop, []}),
    i2c_servo:setPWM(0,0,600),
    i2c_servo:setPWM(1,0,150),
    sharp:detect_obstacle().
%%    receive
%%	{detect_obstacle, true} ->
%%	    motor_stop(),
%%	    T2 = now(),
%%	    trunc(timer:now_diff(T2, T1) * 0.001)
%%    after
%%	T ->
%%	    motor_stop(),
%%	    T
%%    end.

motor_rotate(T, left) ->
    timer:sleep(10),
    chronos:start_timer(motor_timer, stop_motor, T, {motor, stop, []}),
    i2c_servo:setPWM(0,0,150),
    i2c_servo:setPWM(1,0,150);
motor_rotate(T, right) ->
    timer:sleep(10),
    chronos:start_timer(motor_timer, stop_motor, T, {motor, stop, []}),
    i2c_servo:setPWM(0,0,600),
    i2c_servo:setPWM(1,0,600).

%% rotate_compass(Degrees, left) ->
%%     i2c_servo:setPWM(0, 0, 150),
%%     i2c_servo:setPWM(1, 0, 150),
%%     compass:rotate_degrees(Degrees, left),
%%     receive
%% 	{rotate_degrees, true} ->
%% 	    stop()
%%     end;
%% rotate_compass(Degrees, right) ->
%%     i2c_servo:setPWM(0, 0, 600),
%%     i2c_servo:setPWM(1, 0, 600),
%%     compass:rotate_degrees(Degrees, right),
%%     receive
%% 	{rotate_degrees, true} ->
%% 	    stop()
%%     end.

motor_stop() ->
    i2c_servo:setPWM(0,0,0),
    i2c_servo:setPWM(1,0,0).

%%stop(Channel) ->
%%    i2c_servo:setPWM(Channel, 0, 0).
