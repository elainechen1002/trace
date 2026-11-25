using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Threading;

public static class ShoulderPan
{
    private const byte MotorId = 1;
    private const int MaximumStep = 16;
    private const int Speed = 90;
    private const byte Torque = 40;
    private const byte GoalPosition = 42;
    private const byte PresentPosition = 56;

    public static void Run(string port, bool enableMotion, int seconds, int travelDegrees)
    {
        if (travelDegrees < 1 || travelDegrees > 18)
            throw new ArgumentOutOfRangeException("travelDegrees");
        int travel = (int)Math.Floor(travelDegrees * 4096.0 / 360.0);
        using (var bus = new SerialPort(port, 1000000, Parity.None, 8, StopBits.One))
        {
            bus.ReadTimeout = 30;
            bus.WriteTimeout = 150;
            bus.Open();

            int model = Word(Read(bus, 3, 2), 0);
            byte mode = Read(bus, 33, 1)[0];
            byte torque = Read(bus, Torque, 1)[0];
            byte[] limits = Read(bus, 9, 4);
            int minimum = Word(limits, 0);
            int maximum = Word(limits, 2);
            int home = Word(Read(bus, PresentPosition, 2), 0);
            Console.WriteLine("{0}: ID 1, model {1}, mode {2}, torque {3}, position {4}, limits {5}–{6}.",
                port, model, mode, torque, home, minimum, maximum);
            if (!enableMotion)
                return;

            if (model != 777 || mode != 0 || torque != 0 || minimum >= maximum ||
                home - travel < minimum || home + travel > maximum || maximum > 4095)
                throw new InvalidOperationException("Shoulder pan must be a stationary, untorqued STS3215 with room inside its configured limits.");
            if (Word(Read(bus, 58, 2), 0) != 0)
                throw new InvalidOperationException("Shoulder pan is moving.");

            var input = new BlockingCollection<string>(1);
            new Thread(() => {
                try {
                    string line;
                    while ((line = Console.ReadLine()) != null)
                        input.Add(line);
                } finally {
                    input.CompleteAdding();
                }
            }) { IsBackground = true }.Start();

            var clock = Stopwatch.StartNew();
            bool armed = false;
            bool active = false;
            double origin = 0;
            int goal = home;
            int pinchPosition = home;
            try
            {
                while (clock.Elapsed.TotalSeconds < seconds)
                {
                    string sample;
                    if (!input.TryTake(out sample, 450))
                    {
                        if (active)
                            Write(bus, Torque, 0);
                        active = false;
                        armed = false;
                        if (input.IsCompleted)
                            break;
                        continue;
                    }

                    double x;
                    if (!double.TryParse(sample, NumberStyles.Float, CultureInfo.InvariantCulture, out x) ||
                        double.IsNaN(x) || double.IsInfinity(x))
                    {
                        if (active)
                        {
                            Write(bus, Torque, 0);
                            Console.WriteLine("Shoulder pan released.");
                        }
                        active = false;
                        armed = sample == "open";
                        continue;
                    }

                    if (!active)
                    {
                        if (!armed)
                            continue;
                        int current = Word(Read(bus, PresentPosition, 2), 0);
                        if (Math.Abs(current - home) > travel)
                            throw new InvalidOperationException("Arm moved outside the test range.");
                        origin = x;
                        pinchPosition = current;
                        goal = current;
                        // Set acceleration, present position as the goal, time, and speed before torque.
                        Write(bus, 41, 4, Low(goal), High(goal), 0, 0, Low(Speed), High(Speed));
                        active = true; // Attempt release even if the enable acknowledgement is lost.
                        Write(bus, Torque, 1);
                        Console.WriteLine("Shoulder pan enabled.");
                        continue;
                    }

                    int currentPosition = Word(Read(bus, PresentPosition, 2), 0);
                    if (Math.Abs(currentPosition - home) > travel + 8)
                        throw new InvalidOperationException("Shoulder pan left the test range.");
                    double delta = Math.Max(-travel, Math.Min(travel, (x - origin) * travel / 50.0));
                    // Anchor each pinch at the measured position to avoid a jump on re-engage.
                    int target = Math.Max(home - travel, Math.Min(home + travel, pinchPosition + (int)Math.Round(delta)));
                    int next = goal + Math.Max(-MaximumStep, Math.Min(MaximumStep, target - goal));
                    if (next != goal)
                    {
                        Write(bus, GoalPosition, Low(next), High(next));
                        goal = next;
                        Console.WriteLine("Shoulder goal: {0}; measured position: {1}.", goal, currentPosition);
                    }
                }
            }
            finally
            {
                if (active)
                {
                    Write(bus, Torque, 0);
                    Console.WriteLine("Shoulder pan released.");
                }
            }
        }
    }

    private static byte Low(int value) { return (byte)(value & 255); }
    private static byte High(int value) { return (byte)(value >> 8); }
    private static int Word(byte[] bytes, int index) { return bytes[index] | (bytes[index + 1] << 8); }

    private static byte[] Read(SerialPort bus, byte address, byte count)
    {
        return ReadMotor(bus, MotorId, address, count);
    }

    private static void Write(SerialPort bus, byte address, params byte[] values)
    {
        WriteMotor(bus, MotorId, address, values);
    }

    internal static byte[] ReadMotor(SerialPort bus, byte motorId, byte address, byte count)
    {
        return Exchange(bus, motorId, 2, count, new byte[] { address, count });
    }

    internal static void WriteMotor(SerialPort bus, byte motorId, byte address, params byte[] values)
    {
        Exchange(bus, motorId, 3, 0, new byte[] { address }.Concat(values).ToArray());
    }

    private static byte[] Exchange(SerialPort bus, byte motorId, byte instruction, int responseLength, byte[] parameters)
    {
        var request = new List<byte> { 255, 255, motorId, (byte)(2 + parameters.Length), instruction };
        request.AddRange(parameters);
        request.Add((byte)(255 - (request.Skip(2).Sum(value => (int)value) & 255)));
        bus.DiscardInBuffer();
        bus.Write(request.ToArray(), 0, request.Count);

        var received = new List<byte>();
        var clock = Stopwatch.StartNew();
        while (clock.ElapsedMilliseconds < 180)
        {
            try { received.Add((byte)bus.ReadByte()); }
            catch (TimeoutException) { continue; }
            while (received.Count >= 4)
            {
                if (received[0] != 255 || received[1] != 255 || received[2] != motorId ||
                    received[3] != responseLength + 2)
                {
                    received.RemoveAt(0);
                    continue;
                }
                int length = responseLength + 6;
                if (received.Count < length)
                    break;
                var packet = received.Take(length).ToArray();
                if ((packet.Skip(2).Sum(value => (int)value) & 255) != 255)
                {
                    received.RemoveAt(0);
                    continue;
                }
                received.RemoveRange(0, length);
                if (packet.SequenceEqual(request))
                    continue;
                if (packet[4] != 0)
                    throw new InvalidOperationException("Motor " + motorId + " reported status " + packet[4] + ".");
                return packet.Skip(5).Take(responseLength).ToArray();
            }
        }
        throw new TimeoutException("Motor " + motorId + " did not acknowledge instruction " + instruction + ".");
    }
}
