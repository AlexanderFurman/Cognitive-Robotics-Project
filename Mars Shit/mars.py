from pyRDDLGym import RDDLEnv
from pyRDDLGym import ExampleManager
from pyRDDLGym.Policies.Agents import RandomAgent

# get the environment info
EnvInfo = ExampleManager.GetEnvInfo('MarsRover')

# set up the environment class, choose instance 0 because every example has at least one example instance
myEnv = RDDLEnv.RDDLEnv(domain=EnvInfo.get_domain(), instance=EnvInfo.get_instance(0))
# set up the environment visualizer
myEnv.set_visualizer(EnvInfo.get_visualizer())

# set up an aget
agent = RandomAgent(action_space=myEnv.action_space, num_actions=myEnv.NumConcurrentActions)

total_reward = 0
state = myEnv.reset()
for _ in range(myEnv.horizon):
      myEnv.render()
      next_state, reward, done, info = myEnv.step(agent.sample_action())
      total_reward += reward
      state = next_state
      if done:
            break

myEnv.close()