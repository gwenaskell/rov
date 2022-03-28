export type Ok<T> = { isOk: true; value: T };

export type Error = { isOk: false; error: string };

export type Result<T> = Ok<T> | Error;

// helper functions for creating our OK and Error types
export function ok<T>(value: T): Ok<T> {
  return { isOk: true, value: value };
}

export function error(message: string): Error {
  return { isOk: false, error: message };
}