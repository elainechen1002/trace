using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Threading;

public static class IkDrive
{
    private const int Count = 5;
    private const int Neutral = 2048;
    private const int CenterTolerance = 100;
    private const int MaximumStep = 10;
    private const int Speed = 55;

    public static void Run(string port, bool enableMotion, int travel)
    {
        if (travel < 1 || travel > 64)
            throw new ArgumentOutOfRangeException("travel");

        using (var bus = new SerialPort(port, 1000000, Parity.None, 8, StopBits.One))
        {
            bus.ReadTimeout = 30;
            bus.WriteTimeout = 150;
            bus.Open();
            var home = ReadStartingPose(bus, travel);
            Console.WriteLine("SO-101 centered: " + string.Join(", ", home));
            if (!enableMotion)
                return;

            var input = new BlockingCollection<string>(1);
            new Thread(() => ReadInput(input)) { IsBackground = true }.Start();
            var enabled = new bool[Count];
            var goals = (int[])home.Clone();
            bool following = false;
            var log = Stopwatch.StartNew();
            Console.WriteLine("Ready. Show an open hand, then pinch and move. Release to hold.");

            try
            {
                while (!input.IsCompleted)
                {
                    string message;
                    if (!input.TryTake(out message, 500) || message == "HOLD")
                    {
                        if (following)
                        {
                            Hold(bus, enabled, goals, home, travel);
                            Console.WriteLine("Holding position.");
                        }
                        following = false;
                        continue;
                    }

                    int[] offsets = ParseOffsets(message, travel);
                    int[] actual;
                    if (enabled[0])
                        actual = ReadPositions(bus);
                    else
                        actual = home = ReadStartingPose(bus, travel);
                    if (actual.Where((value, i) => Math.Abs(value - home[i]) > travel + 20).Any())
                        throw new InvalidOperationException("A joint left the allowed test range.");

                    if (!enabled[0])
                    {
                        for (int i = 0; i < Count; i++)
                        {
                            SetGoal(bus, i, actual[i]);
                            goals[i] = actual[i];
                        }
                        for (int i = 0; i < Count; i++)
                        {
                            enabled[i] = true;
                            ShoulderPan.WriteMotor(bus, (byte)(i + 1), 40, 1);
                        }
                        Console.WriteLine("IK motion enabled.");
                    }

                    following = true;
                    for (int i = 0; i < Count; i++)
                    {
                        int desired = home[i] + offsets[i];
                        int goal = actual[i] + Math.Max(-MaximumStep, Math.Min(MaximumStep, desired - actual[i]));
                        goal = Math.Max(home[i] - travel, Math.Min(home[i] + travel, goal));
                        if (goals[i] != goal)
                        {
                            SetGoal(bus, i, goal);
                            goals[i] = goal;
                        }
                    }
                    if (log.ElapsedMilliseconds > 1000)
                    {
                        Console.WriteLine("Measured: " + string.Join(", ", actual));
                        log.Restart();
                    }
                }
            }
            finally
            {
                if (enabled.Any(value => value))
                {
                    Hold(bus, enabled, goals, home, travel);
                    Console.WriteLine("Motion stopped. Motors are holding the pose.");
                }
            }
        }
    }

    private static int[] ReadStartingPose(SerialPort bus, int travel)
    {
        int[] positions = ReadPositions(bus);
        for (int i = 0; i < Count; i++)
        {
            byte id = (byte)(i + 1);
            byte[] limits = ShoulderPan.ReadMotor(bus, id, 9, 4);
            if (Word(ShoulderPan.ReadMotor(bus, id, 3, 2), 0) != 777 ||
                ShoulderPan.ReadMotor(bus, id, 33, 1)[0] != 0 ||
                ShoulderPan.ReadMotor(bus, id, 40, 1)[0] != 0 ||
                Word(ShoulderPan.ReadMotor(bus, id, 58, 2), 0) != 0 ||
                Math.Abs(positions[i] - Neutral) > CenterTolerance ||
                positions[i] - travel - 20 < Word(limits, 0) ||
                positions[i] + travel + 20 > Word(limits, 2))
                throw new InvalidOperationException("Motor " + id + " failed the centered, torque-off preflight.");
        }
        return positions;
    }

    private static void ReadInput(BlockingCollection<string> input)
    {
        try
        {
            string message;
            while ((message = Console.ReadLine()) != null)
                input.Add(message);
        }
        finally
        {
            input.CompleteAdding();
        }
    }

    private static int[] ParseOffsets(string message, int travel)
    {
        int[] offsets = message.Split(',').Select(int.Parse).ToArray();
        if (offsets.Length != Count || offsets.Any(offset => offset < -travel || offset > travel))
            throw new InvalidOperationException("Invalid IK joint offsets.");
        return offsets;
    }

    private static int[] ReadPositions(SerialPort bus)
    {
        return Enumerable.Range(1, Count)
            .Select(id => Word(ShoulderPan.ReadMotor(bus, (byte)id, 56, 2), 0))
            .ToArray();
    }

    private static int Word(byte[] data, int start)
    {
        return data[start] | (data[start + 1] << 8);
    }

    private static void SetGoal(SerialPort bus, int joint, int position)
    {
        ShoulderPan.WriteMotor(bus, (byte)(joint + 1), 41, 4,
            (byte)(position & 255), (byte)(position >> 8), 0, 0, Speed, 0);
    }

    private static void Hold(SerialPort bus, bool[] enabled, int[] goals, int[] home, int travel)
    {
        for (int i = 0; i < Count; i++)
        {
            if (!enabled[i])
                continue;
            int position = Word(ShoulderPan.ReadMotor(bus, (byte)(i + 1), 56, 2), 0);
            int goal = Math.Max(home[i] - travel, Math.Min(home[i] + travel, position));
            SetGoal(bus, i, goal);
            goals[i] = goal;
        }
    }
}
