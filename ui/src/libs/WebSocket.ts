export default class WS {
  private static conn?: WebSocket;
  private static url?: string;


  private static callback?: (payload: string) => void;

  private static connected = false;

  public static open_connection(callback: (payload: string) => void): void {
    if (WS.url) {
      return;
    }
    // if (window.location.host.includes('localhost')) {
    //   console.log("no connection opened");
    //   return;
    // }
    WS.url = "ws://" + window.location.hostname+':8000/socket';
    console.log(WS.url);
    WS.callback = callback;
    console.log("establishing socket connection")
    setTimeout(WS.connect, 0); // async
  }

  private static connect(): void {
    if (WS.conn) {
      throw Error("err: socket already opened");
    }
    console.log("connecting")
    let conn = new WebSocket(WS.url as string);

    conn.onopen = (e: Event) => {
      console.log("socket opened");
      WS.connected = true;
      WS.conn = conn;
    };

    conn.onmessage = (e: MessageEvent) => {
      WS.callback?.(e.data);
    }

    conn.onerror = (e: Event) => {
      console.log("socket errored");
      WS.connected = false;
      WS.conn = undefined;
      conn.close();
    };

    conn.onclose = (e: Event) => {
      console.log("socket closed. Retrying to connect in 2s");
      WS.connected = false;
      WS.conn = undefined;
      setTimeout(WS.connect, 2000); // retry
    };
  }

  public static send(payload: string) {
    WS.conn?.send(payload);
  }
}

// type ConnectButtonClicked = { type: "ConnectButtonClicked" };
// type InputUpdated = { type: "InputUpdated"; value: string };
// type SuccessfulConnection = { type: "SuccessfulConnection"; value: WebSocket };
// type FailedConnection = { type: "FailedConnection"; value: string };
// type DisconnectButtonClicked = { type: "DisconnectButtonClicked" };
// type SocketClosed = { type: "SocketClosed" };

// type Action =
//   | InputUpdated
//   | ConnectButtonClicked
//   | DisconnectButtonClicked
//   | SuccessfulConnection
//   | FailedConnection
//   | SocketClosed;
