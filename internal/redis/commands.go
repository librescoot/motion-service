package redis

import (
	"context"
	"log/slog"
	"strings"
)

// CommandHandler handles commands from the scooter:motion queue
type CommandHandler struct {
	client  *Client
	log     *slog.Logger
	handler func(string, string)
}

// NewCommandHandler creates a new CommandHandler
func NewCommandHandler(client *Client, log *slog.Logger, handler func(string, string)) *CommandHandler {
	return &CommandHandler{
		client:  client,
		log:     log,
		handler: handler,
	}
}

// Run starts the command handler loop
func (h *CommandHandler) Run(ctx context.Context) error {
	h.log.Info("starting command handler")

	for {
		select {
		case <-ctx.Done():
			h.log.Info("command handler stopped")
			return nil

		default:
			cmd, err := h.client.BRPop(ctx, "scooter:motion")
			if err != nil {
				if err == context.Canceled {
					return nil
				}
				h.log.Error("error reading from scooter:motion", "error", err)
				continue
			}

			h.log.Info("received command", "command", cmd)
			h.handleCommand(cmd)
		}
	}
}

// handleCommand parses and handles a command string
func (h *CommandHandler) handleCommand(cmd string) {
	parts := strings.SplitN(cmd, ":", 2)
	action := parts[0]
	param := ""
	if len(parts) > 1 {
		param = parts[1]
	}

	h.handler(action, param)
}