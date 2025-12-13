import argparse
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style

from grid_navigation_env import GridNavigationEnv
from grid_dqn_agent import Agent, AgentConfig

style.use('ggplot')


def _parse_coord(text: str):
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Coordinate must be in the form x,y")
    try:
        return int(parts[0]), int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Coordinate values must be integers") from exc


def train(args):
    env = GridNavigationEnv(args.map, args.start, args.goal, max_steps=args.max_steps)

    cfg = AgentConfig(
        state_size=env.state_size,
        action_size=env.action_size,
        hidden_size=args.hidden_size,
        lr=args.lr,
        gamma=args.gamma,
        batch_size=args.batch_size,
        epsilon_start=args.epsilon_start,
        epsilon_end=args.epsilon_end,
        epsilon_decay=args.epsilon_decay,
    )
    agent = Agent(cfg)

    if args.model_in and os.path.isfile(args.model_in):
        agent.load(args.model_in)
        # Optionally lower epsilon when resuming
        agent.epsilon = max(args.epsilon_end, args.resume_epsilon)
        print(f"Loaded weights from {args.model_in}; starting epsilon {agent.epsilon:.3f}")

    best_reward = -float("inf")
    os.makedirs(os.path.dirname(args.model_out) or ".", exist_ok=True)

    # Tracking for live plots
    episode_rewards = []
    success_rates = []
    recent_successes = []
    window_size = 100  # for moving average

    # Setup live plotting if enabled
    if args.plot_live:
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle('Training Progress', fontsize=14)

    for episode in range(1, args.episodes + 1):
        state = env.reset(random_goal=args.randomize_goals)
        done = False
        total_reward = 0.0

        while not done:
            action = agent.get_action(state)
            next_state, reward, done, _ = env.step(action)
            agent.train_short_memory(state, action, reward, next_state, done)
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward

            # Optional live rendering during training
            if args.render_training and (episode % args.render_every == 0):
                env.render(pause=args.render_pause)

        agent.train_long_memory()
        agent.n_games += 1

        # Track stats
        episode_rewards.append(total_reward)
        success = int(env.pos == env.goal)
        recent_successes.append(success)
        if len(recent_successes) > window_size:
            recent_successes.pop(0)
        success_rate = sum(recent_successes) / len(recent_successes) * 100
        success_rates.append(success_rate)

        if total_reward > best_reward:
            best_reward = total_reward
            agent.save(args.model_out)

        if episode % args.log_every == 0 or episode == 1:
            print(
                f"Episode {episode}/{args.episodes} | Reward {total_reward:.2f} | "
                f"Best {best_reward:.2f} | Success Rate {success_rate:.1f}% | Epsilon {agent.epsilon:.3f}"
            )

        # Update live plot
        if args.plot_live and episode % args.plot_update_every == 0:
            ax1.clear()
            ax2.clear()

            # Plot rewards
            ax1.plot(episode_rewards, label='Reward', alpha=0.6, color='blue')
            if len(episode_rewards) >= 10:
                smooth_rewards = np.convolve(episode_rewards, np.ones(10)/10, mode='valid')
                ax1.plot(range(9, len(episode_rewards)), smooth_rewards, label='Avg (10 ep)', color='red', linewidth=2)
            ax1.set_xlabel('Episode')
            ax1.set_ylabel('Total Reward')
            ax1.set_title('Episode Rewards')
            ax1.legend(loc='lower right')
            ax1.grid(True, alpha=0.3)

            # Plot success rate
            ax2.plot(success_rates, label=f'Success Rate ({window_size} ep window)', color='green', linewidth=2)
            ax2.axhline(y=50, color='gray', linestyle='--', alpha=0.5)
            ax2.set_xlabel('Episode')
            ax2.set_ylabel('Success Rate (%)')
            ax2.set_title('Goal Achievement Rate')
            ax2.set_ylim([0, 105])
            ax2.legend(loc='lower right')
            ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.pause(0.01)

    print(f"Training done. Best reward {best_reward:.2f}. Model saved to {args.model_out}")
    print(f"Final success rate: {success_rates[-1]:.1f}%")

    # Save final plot
    if args.plot_live:
        if args.save_plot:
            plt.savefig(args.save_plot, dpi=150, bbox_inches='tight')
            print(f"Training plot saved to {args.save_plot}")
        plt.ioff()
        plt.show()

    # Run final episode and save as GIF
    print("\nRunning final demonstration episode...")
    state = env.reset(random_goal=args.randomize_goals)
    done = False
    final_reward = 0.0
    
    while not done:
        action = agent.get_action(state)  # Greedy (epsilon=0)
        next_state, reward, done, _ = env.step(action)
        env.render(pause=0.01)
        state = next_state
        final_reward += reward
    
    # Save final episode as GIF
    gif_path = args.model_out.replace('.pth', '_final_episode.gif')
    if env._frames:
        env.save_gif(gif_path, fps=10)
        print(f"Final episode GIF saved to {gif_path}")
    print(f"Final episode reward: {final_reward:.2f}")


def main():
    parser = argparse.ArgumentParser(description="Train a DQN agent on grid maps")
    parser.add_argument("--map", default="map1.txt", help="Path to map text file (0 free, 1 obstacle)")
    parser.add_argument("--start", type=_parse_coord, default="0,0", help="Start coordinate x,y")
    parser.add_argument("--goal", type=_parse_coord, default="8,8", help="Goal coordinate x,y (ignored if --randomize_goals)")
    parser.add_argument("--randomize_goals", action="store_true", help="Sample random goal each episode for better generalization")
    parser.add_argument("--episodes", type=int, default=1000, help="Number of training episodes")
    parser.add_argument("--max_steps", type=int, default=200, help="Max steps per episode")
    parser.add_argument("--hidden_size", type=int, default=256, help="Hidden size for Q-network")
    parser.add_argument("--lr", type=float, default=5e-4, help="Learning rate")
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    parser.add_argument("--batch_size", type=int, default=512, help="Batch size for replay")
    parser.add_argument("--epsilon_start", type=float, default=1.0, help="Initial epsilon")
    parser.add_argument("--epsilon_end", type=float, default=0.01, help="Final epsilon")
    parser.add_argument("--epsilon_decay", type=float, default=0.998, help="Epsilon decay per action")
    parser.add_argument("--model_in", default=None, help="Optional path to existing model to continue training")
    parser.add_argument("--resume_epsilon", type=float, default=0.1, help="Epsilon to start from when resuming")
    parser.add_argument("--model_out", default="./models/grid_dqn.pth", help="Path to save best model")
    parser.add_argument("--log_every", type=int, default=10, help="Logging frequency")
    parser.add_argument("--render_training", action="store_true", help="Render live frames during training (slow)")
    parser.add_argument("--render_every", type=int, default=50, help="Render every N episodes when --render_training is set")
    parser.add_argument("--render_pause", type=float, default=0.05, help="Pause between frames when rendering during training")
    parser.add_argument("--plot_live", action="store_true", help="Show live training plots (reward and success rate)")
    parser.add_argument("--plot_update_every", type=int, default=10, help="Update plot every N episodes")
    parser.add_argument("--save_plot", type=str, default=None, help="Path to save final training plot (e.g., training_progress.png)")

    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
