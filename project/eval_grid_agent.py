import argparse
import matplotlib.pyplot as plt

from grid_navigation_env import GridNavigationEnv
from grid_dqn_agent import Agent, AgentConfig


def _parse_coord(text: str):
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Coordinate must be in the form x,y")
    try:
        return int(parts[0]), int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Coordinate values must be integers") from exc


def evaluate(args):
    """Test the trained agent over a number of episodes and report performance."""
    env = GridNavigationEnv(args.map, args.start, args.goal, max_steps=args.max_steps)
    cfg = AgentConfig(
        state_size=env.state_size,
        action_size=env.action_size,
        hidden_size=args.hidden_size,
    )
    agent = Agent(cfg)
    agent.load(args.model)
    agent.epsilon = 0.0
    agent.cfg.epsilon_end = 0.0
    agent.cfg.epsilon_decay = 0.0

    successes = 0
    total_reward = 0.0
    total_steps = 0
    episode_rewards = []

    for ep in range(args.episodes):
        state = env.reset(random_goal=args.randomize_goals)
        done = False
        episode_reward = 0.0
        steps = 0

        while not done:
            # Greedy action selection
            action = agent.get_action(state)
            next_state, reward, done, _ = env.step(action)
            state = next_state
            episode_reward += reward
            steps += 1

            if args.render:
                env.render(pause=args.pause)

        # If rendering was on, save GIF per episode
        if args.render and args.gif:
            gif_path = args.gif if args.episodes == 1 else f"{args.gif.rsplit('.',1)[0]}_ep{ep+1}.gif"
            env.save_gif(gif_path, fps=args.fps)

        successes += int(env.pos == env.goal)
        total_reward += episode_reward
        total_steps += steps
        episode_rewards.append(episode_reward)

    print(f"Episodes: {args.episodes}")
    print(f"Success rate: {successes / args.episodes:.2%}")
    print(f"Avg reward: {total_reward / args.episodes:.2f}")
    print(f"Avg steps: {total_steps / args.episodes:.1f}")

    # Plot reward curve
    if args.plot_rewards:
        plt.figure(figsize=(10, 6))
        plt.plot(range(1, len(episode_rewards) + 1), episode_rewards, marker='o', linestyle='-', linewidth=2, markersize=4)
        plt.axhline(y=sum(episode_rewards) / len(episode_rewards), color='r', linestyle='--', label=f'Mean: {sum(episode_rewards) / len(episode_rewards):.2f}')
        plt.xlabel('Episode', fontsize=12)
        plt.ylabel('Total Reward', fontsize=12)
        plt.title('Evaluation Reward Curve', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        if args.save_plot:
            plt.savefig(args.save_plot, dpi=150)
            print(f"Reward plot saved to {args.save_plot}")
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained grid DQN agent")
    parser.add_argument("--map", default="map1.txt", help="Path to map text file")
    parser.add_argument("--start", type=_parse_coord, default="0,0", help="Start coordinate x,y")
    parser.add_argument("--goal", type=_parse_coord, default="8,8", help="Goal coordinate x,y (ignored if --randomize_goals)")
    parser.add_argument("--randomize_goals", action="store_true", help="Sample random goal each episode")
    parser.add_argument("--max_steps", type=int, default=200, help="Max steps per episode")
    parser.add_argument("--hidden_size", type=int, default=128, help="Hidden size (must match training)")
    parser.add_argument("--episodes", type=int, default=20, help="Number of evaluation episodes")
    parser.add_argument("--model", default="./models/grid_dqn.pth", help="Path to trained model")
    parser.add_argument("--render", action="store_true", help="Show live navigation with matplotlib")
    parser.add_argument("--pause", type=float, default=0.05, help="Pause between frames when rendering")
    parser.add_argument("--gif", type=str, default=None, help="Path to save rendered episode as GIF (requires --render)")
    parser.add_argument("--fps", type=int, default=5, help="FPS for GIF export")
    parser.add_argument("--plot_rewards", action="store_true", help="Plot reward curve after evaluation")
    parser.add_argument("--save_plot", type=str, default=None, help="Path to save reward plot (e.g., rewards.png)")

    args = parser.parse_args()
    evaluate(args)


if __name__ == "__main__":
    main()
