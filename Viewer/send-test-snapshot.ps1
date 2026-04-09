param(
    [string]$HostName = "127.0.0.1",
    [int]$Port = 4242
)

$snapshot = @{
    walls = @(
        @{
            start = @{ x = -2.0; y = -1.5 }
            end = @{ x = 2.0; y = -1.5 }
            confidence = 6.0
        },
        @{
            start = @{ x = 2.0; y = -1.5 }
            end = @{ x = 2.0; y = 1.5 }
            confidence = 5.0
        },
        @{
            start = @{ x = 2.0; y = 1.5 }
            end = @{ x = -2.0; y = 1.5 }
            confidence = 4.0
        },
        @{
            start = @{ x = -2.0; y = 1.5 }
            end = @{ x = -2.0; y = -1.5 }
            confidence = 5.0
        },
        @{
            start = @{ x = -0.5; y = -1.5 }
            end = @{ x = -0.5; y = -0.2 }
            confidence = 3.0
        },
        @{
            start = @{ x = 0.8; y = 0.3 }
            end = @{ x = 1.7; y = 1.1 }
            confidence = 2.0
        }
    )
} | ConvertTo-Json -Depth 6 -Compress

$payload = [System.Text.Encoding]::UTF8.GetBytes($snapshot + "`n")
$client = [System.Net.Sockets.TcpClient]::new()

try
{
    $client.Connect($HostName, $Port)
    $stream = $client.GetStream()
    $stream.Write($payload, 0, $payload.Length)
    $stream.Flush()
}
finally
{
    $client.Dispose()
}
