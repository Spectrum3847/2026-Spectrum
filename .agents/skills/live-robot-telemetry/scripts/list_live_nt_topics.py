#!/usr/bin/env python3
"""List currently published topics from a live robot NT4 server."""

from __future__ import annotations

import argparse
from nt_live_common import compile_and_run


JAVA_SOURCE = r'''
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.TopicInfo;
import java.util.EnumSet;
import java.util.Map;
import java.util.TreeMap;

public class ListLiveNtTopics {
  private static String jsonEscape(String value) {
    return value.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private static boolean waitConnected(NetworkTableInstance inst, double timeoutSeconds) throws Exception {
    long deadline = System.nanoTime() + (long)(timeoutSeconds * 1_000_000_000L);
    while (System.nanoTime() < deadline) {
      if (inst.isConnected()) {
        return true;
      }
      Thread.sleep(50);
    }
    return inst.isConnected();
  }

  public static void main(String[] args) throws Exception {
    String host = args[0];
    int port = Integer.parseInt(args[1]);
    double timeout = Double.parseDouble(args[2]);
    String filter = args[3].toLowerCase();
    boolean json = Boolean.parseBoolean(args[4]);

    NetworkTableInstance inst = NetworkTableInstance.create();
    inst.startClient4("codex-live-topic-list");
    inst.setServer(host, port);

    if (!waitConnected(inst, timeout)) {
      System.err.println("Timed out waiting for NT4 connection to " + host + ":" + port);
      inst.close();
      System.exit(2);
    }

    final Object lock = new Object();
    Map<String, String> topics = new TreeMap<>();
    inst.addListener(new String[]{""}, EnumSet.of(NetworkTableEvent.Kind.kPublish, NetworkTableEvent.Kind.kImmediate), event -> {
      TopicInfo info = event.topicInfo;
      if (info != null) {
        String name = info.name;
        if (filter.isEmpty() || name.toLowerCase().contains(filter)) {
          synchronized (lock) {
            topics.put(name, info.typeStr);
          }
        }
      }
    });
    Thread.sleep(Math.max(250L, (long)(timeout * 1000.0)));

    Map<String, String> topicSnapshot;
    synchronized (lock) {
      topicSnapshot = new TreeMap<>(topics);
    }

    if (json) {
      System.out.println("{\"host\":\"" + jsonEscape(host) + "\",\"port\":" + port + ",\"topics\":[");
      int i = 0;
      for (Map.Entry<String, String> topic : topicSnapshot.entrySet()) {
        System.out.print("  {\"name\":\"" + jsonEscape(topic.getKey()) + "\",\"type\":\"" + jsonEscape(topic.getValue()) + "\"}");
        System.out.println(++i == topicSnapshot.size() ? "" : ",");
      }
      System.out.println("]}");
    } else {
      System.out.println("Connected to " + host + ":" + port);
      System.out.println("Topics: " + topicSnapshot.size());
      for (Map.Entry<String, String> topic : topicSnapshot.entrySet()) {
        System.out.println(topic.getKey() + "\t" + topic.getValue());
      }
    }
    inst.close();
  }
}
'''


def main() -> None:
    """List currently published topics from a live NT4 server."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.80.44.2", help="Robot IP or hostname")
    parser.add_argument("--port", type=int, default=5810, help="NT4 port")
    parser.add_argument("--timeout", type=float, default=5.0, help="Connection timeout in seconds")
    parser.add_argument("--filter", default="", help="Only show topics containing this text")
    parser.add_argument("--json", action="store_true", help="Print JSON")
    args = parser.parse_args()

    if args.timeout < 0:
        parser.error("--timeout must be non-negative")

    raise SystemExit(
        compile_and_run(
            JAVA_SOURCE,
            "ListLiveNtTopics",
            [args.host, str(args.port), str(args.timeout), args.filter, str(args.json).lower()],
        )
    )


if __name__ == "__main__":
    main()
