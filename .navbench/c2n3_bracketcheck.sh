#!/usr/bin/env bash
# Does bracketing actually stop 'nav2_' matching a helper named
# c2nav2_up.sh? Test it rather than assume it.
#
# Two DIFFERENT things get conflated here:
#   (a) the pattern matching its OWN TEXT -- the invariant ros_clean.sh's
#       header states, and what bracketing fixes;
#   (b) the pattern matching some other process whose command line merely
#       CONTAINS the matched substring -- which bracketing cannot fix,
#       because 'nav[2]_' and 'nav2_' match exactly the same strings.
echo "=== (a) does the pattern match its own TEXT? ==="
for pat in 'nav2_' 'nav[2]_'; do
  n=$(printf '%s\n' "$pat" | grep -cE "$pat")
  printf '  pattern %-10s matches the literal text "%s": %s\n' "$pat" "$pat" "$n"
done

echo
echo "=== (b) does the pattern match a command line containing nav2_ ? ==="
for pat in 'nav2_' 'nav[2]_'; do
  for target in 'bash /home/x/.navbench/c2nav2_up.sh' \
                'ros2 launch gazebo_models nav.launch.py params_file:=/home/x/nav2_params.yaml' \
                'bash /home/x/.navbench/c2n3_up_nav.sh' \
                'python3 /home/x/docs/data/c2nav3_capture.py' \
                '/opt/ros/jazzy/lib/nav2_controller/controller_server'; do
    n=$(printf '%s\n' "$target" | grep -cE "$pat")
    [ "$n" = 1 ] && printf '  %-10s MATCHES  %s\n' "$pat" "$target"
  done
done
echo
echo "  -> if both patterns match the same lines above, bracketing does NOT"
echo "     protect a caller whose own name contains the substring."
